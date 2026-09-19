#include "imu_attitude.h"
#include <math.h>
#include <string.h>
/* Initialization marker only; does not validate arbitrary/dangling pointers. */
#define IMU_TAG 0x494D5531U

/* Internal: restart_calibration clears all Welford state through this helper.
 */
static void calibration_reset(imu_attitude_calibration_t *calibration) {
  memset(calibration, 0, sizeof(*calibration));
}

/* Internal, called by imu_attitude_update during calibration only. Welford
 * accumulation avoids subtracting large near-equal sums to estimate variance.
 */
static void calibration_add(imu_attitude_calibration_t *calibration,
                            const imu_attitude_sample_t *sample) {
  size_t axis;
  const kalman_real_t next_count = (kalman_real_t)(calibration->count + 1U);

  ++calibration->count;
  for (axis = 0U; axis < 3U; ++axis) {
    const kalman_real_t accel = (kalman_real_t)sample->accel_mps2[axis];
    const kalman_real_t gyro = (kalman_real_t)sample->gyro_rads[axis];
    const kalman_real_t accel_delta = accel - calibration->accel_mean[axis];
    const kalman_real_t gyro_delta = gyro - calibration->gyro_mean[axis];

    calibration->accel_mean[axis] += accel_delta / next_count;
    calibration->gyro_mean[axis] += gyro_delta / next_count;
    calibration->accel_m2[axis] +=
        accel_delta * (accel - calibration->accel_mean[axis]);
    calibration->gyro_m2[axis] +=
        gyro_delta * (gyro - calibration->gyro_mean[axis]);
  }
}

/* Internal, called by imu_attitude_update when the calibration window fills.
 * Compare mean norms and worst-axis sample deviations; this is a heuristic,
 * not proof of rest (constant slow rotation can pass these thresholds). */
static bool calibration_is_stationary(
    const imu_attitude_calibration_t *calibration,
    const imu_attitude_config_t *config, kalman_real_t gravity_mps2,
    kalman_real_t *accel_norm_mps2, kalman_real_t *gyro_norm_rads,
    kalman_real_t *max_accel_std_mps2, kalman_real_t *max_gyro_std_rads) {
  kalman_real_t accel_squared = (kalman_real_t)0;
  kalman_real_t gyro_squared = (kalman_real_t)0;
  kalman_real_t max_accel_variance = (kalman_real_t)0;
  kalman_real_t max_gyro_variance = (kalman_real_t)0;
  kalman_real_t gravity_error;
  size_t axis;

  if (calibration->count < 2U) {
    return false;
  }
  for (axis = 0U; axis < 3U; ++axis) {
    const kalman_real_t divisor = (kalman_real_t)(calibration->count - 1U);
    const kalman_real_t accel_variance = calibration->accel_m2[axis] / divisor;
    const kalman_real_t gyro_variance = calibration->gyro_m2[axis] / divisor;

    if (accel_variance > max_accel_variance) {
      max_accel_variance = accel_variance;
    }
    if (gyro_variance > max_gyro_variance) {
      max_gyro_variance = gyro_variance;
    }
    accel_squared +=
        calibration->accel_mean[axis] * calibration->accel_mean[axis];
    gyro_squared += calibration->gyro_mean[axis] * calibration->gyro_mean[axis];
  }

  /* Report physical units, then apply all four independent acceptance limits.
   */
  *accel_norm_mps2 = KALMAN_REAL_SQRT(accel_squared);
  *gyro_norm_rads = KALMAN_REAL_SQRT(gyro_squared);
  *max_accel_std_mps2 = KALMAN_REAL_SQRT(max_accel_variance);
  *max_gyro_std_rads = KALMAN_REAL_SQRT(max_gyro_variance);
  gravity_error = *accel_norm_mps2 - gravity_mps2;
  if (gravity_error < (kalman_real_t)0) {
    gravity_error = -gravity_error;
  }

  return gravity_error <= (kalman_real_t)config->gravity_tolerance_mps2 &&
         *gyro_norm_rads <= (kalman_real_t)config->max_gyro_mean_rads &&
         *max_accel_std_mps2 <= (kalman_real_t)config->max_accel_std_mps2 &&
         *max_gyro_std_rads <= (kalman_real_t)config->max_gyro_std_rads;
}

/* Internal, called by imu_attitude_update after a stationary window >=2.
 * Variance of the sample mean assumes independent noise; a configured floor
 * avoids declaring unrealistically perfect bias for a quiet/quantized window.
 */
static void calibration_get_gyro_bias_variance(
    const imu_attitude_calibration_t *calibration,
    const imu_attitude_config_t *config,
    kalman_real_t gyro_bias_variance_rads2[3]) {
  const kalman_real_t sample_divisor = (kalman_real_t)(calibration->count - 1U);
  const kalman_real_t mean_divisor = (kalman_real_t)calibration->count;
  const kalman_real_t variance_floor =
      (kalman_real_t)config->bias_std_floor_rads *
      (kalman_real_t)config->bias_std_floor_rads;
  size_t axis;

  for (axis = 0U; axis < 3U; ++axis) {
    const kalman_real_t sample_variance =
        calibration->gyro_m2[axis] / sample_divisor;
    const kalman_real_t mean_variance = sample_variance / mean_divisor;
    gyro_bias_variance_rads2[axis] =
        mean_variance > variance_floor ? mean_variance : variance_floor;
  }
}

/* Internal, called by imu_attitude_update only in RUNNING with jerk enabled.
 * Return a dimensionless R multiplier; reject is a separate hard-gate output.
 * During history fill, use multiplier=1 and jerk=0, not a stationarity claim.
 */
static float accel_dynamics_quality(imu_attitude_dynamics_t *dynamics,
                                    const imu_attitude_config_t *config,
                                    const float accel_mps2[3], int64_t time_us,
                                    int64_t dt_us, bool *reject,
                                    float *jerk_mps3) {
  float squared = 0.0f;
  float mean[3];
  *reject = false;
  *jerk_mps3 = 0.0f;
  if (dt_us <= 0 || dt_us > config->dynamics_gap_us) {
    memset(dynamics, 0, sizeof(*dynamics));
  }
  /* Maintain the sum in O(axes) work instead of rescanning the entire ring. */
  for (size_t axis = 0; axis < 3; ++axis) {
    if (dynamics->window_count == config->average_samples)
      dynamics->sum[axis] -= dynamics->window[dynamics->window_index][axis];
    dynamics->window[dynamics->window_index][axis] = accel_mps2[axis];
    dynamics->sum[axis] += accel_mps2[axis];
  }
  dynamics->window_index =
      (dynamics->window_index + 1U) % config->average_samples;
  if (dynamics->window_count < config->average_samples)
    ++dynamics->window_count;
  if (dynamics->window_count != config->average_samples)
    return 1.0f;
  for (size_t axis = 0; axis < 3; ++axis)
    mean[axis] = dynamics->sum[axis] / (float)config->average_samples;
  /* Subtract integer timestamps before conversion to avoid long-run float
   * clock precision loss. Rotation of gravity also contributes to this jerk. */
  if (dynamics->mean_count == config->difference_samples) {
    const float elapsed =
        (float)(time_us - dynamics->mean_time_us[dynamics->mean_index]) *
        1.0e-6f;
    if (elapsed > 0.0f) {
      for (size_t axis = 0; axis < 3; ++axis) {
        const float delta =
            (mean[axis] - dynamics->mean_history[dynamics->mean_index][axis]) /
            elapsed;
        squared += delta * delta;
      }
      *jerk_mps3 = sqrtf(squared);
    }
  } else {
    ++dynamics->mean_count;
  }
  memcpy(dynamics->mean_history[dynamics->mean_index], mean, sizeof(mean));
  dynamics->mean_time_us[dynamics->mean_index] = time_us;
  dynamics->mean_index =
      (dynamics->mean_index + 1U) % config->difference_samples;
  if (*jerk_mps3 >= config->jerk_hard_mps3) {
    *reject = true;
    return 1.0f;
  }
  if (*jerk_mps3 <= config->jerk_soft_mps3) {
    return 1.0f;
  }
  /* Smoothly reduce trust below the hard boundary; no correction at/above it.
   */
  const float ratio = (*jerk_mps3 - config->jerk_soft_mps3) /
                      (config->jerk_hard_mps3 - config->jerk_soft_mps3);
  return 1.0f + (config->jerk_max_variance_multiplier - 1.0f) * ratio * ratio;
}

/* Internal: notify_gap, filter, seed and update guard initialized contexts. */
static bool valid(const imu_attitude_t *c) { return c && c->tag == IMU_TAG; }
/* Internal: imu_attitude_init rejects NaN/Inf as well as negative limits. */
static bool nonnegative(kalman_real_t v) { return isfinite(v) && v >= 0; }

/* External only: construct a value configuration; no resources are acquired. */
imu_attitude_config_t imu_attitude_config_default(void) {
  imu_attitude_config_t c = {
      .filter = KALMAN_ATTITUDE_CONFIG_DEFAULT(),
      .warmup_samples = 500,
      .calibration_samples = 2000,
      .max_accel_std_mps2 = 0.15f,
      .max_gyro_std_rads = 0.01f,
      .max_gyro_mean_rads = 0.15f,
      .gravity_tolerance_mps2 = 0.5f,
      .bias_std_floor_rads = 0.000872664626f,
      .jerk_enabled = true,
      .average_samples = 20,
      .difference_samples = 10,
      .jerk_soft_mps3 = 15.0f,
      .jerk_hard_mps3 = 60.0f,
      .jerk_max_variance_multiplier = 25.0f,
      .calibration_gap_us = 1500,
      .dynamics_gap_us = 2000,
      .integrate_across_gaps = true,
      .skip_saturated = false,
  };
  return c;
}
/* Internal: init, notify_gap and update discard the candidate calibration.
 * Keep the filter and timestamp intact; this is not a running-state reset. */
static void restart_calibration(imu_attitude_t *c) {
  calibration_reset(&c->calibration);
  c->warmup_remaining = c->config.warmup_samples;
  c->state =
      c->warmup_remaining ? IMU_ATTITUDE_WARMUP : IMU_ATTITUDE_CALIBRATING;
}
/* External only: validate policy and a temporary filter before committing any
 * caller-owned state. kalman_attitude_init supplies estimator validation. */
kalman_status_t imu_attitude_init(imu_attitude_t *c,
                                  const imu_attitude_config_t *config) {
  if (!c || !config || config->calibration_samples < 2 ||
      config->calibration_samples > 1000000 || !config->calibration_gap_us ||
      !config->dynamics_gap_us || !config->average_samples ||
      config->average_samples > IMU_ATTITUDE_MAX_AVERAGE ||
      !config->difference_samples ||
      config->difference_samples > IMU_ATTITUDE_MAX_DIFFERENCE ||
      !nonnegative(config->max_accel_std_mps2) ||
      !nonnegative(config->max_gyro_std_rads) ||
      !nonnegative(config->max_gyro_mean_rads) ||
      !nonnegative(config->gravity_tolerance_mps2) ||
      !nonnegative(config->bias_std_floor_rads) ||
      !isfinite(config->bias_std_floor_rads * config->bias_std_floor_rads) ||
      !nonnegative(config->jerk_soft_mps3) ||
      !isfinite(config->jerk_hard_mps3) ||
      config->jerk_hard_mps3 <= config->jerk_soft_mps3 ||
      !isfinite(config->jerk_max_variance_multiplier) ||
      config->jerk_max_variance_multiplier < 1)
    return KALMAN_ERROR_INVALID_ARGUMENT;
  kalman_attitude_t filter;
  kalman_status_t status = kalman_attitude_init(&filter, &config->filter);
  if (status != KALMAN_OK)
    return status;
  /* Copy configuration first to support re-init from c->config. */
  imu_attitude_config_t copy = *config;
  memset(c, 0, sizeof(*c));
  c->config = copy;
  c->filter = filter;
  c->tag = IMU_TAG;
  restart_calibration(c);
  return KALMAN_OK;
}
/* External only: record missing acquisition without inventing sensor data. */
void imu_attitude_notify_gap(imu_attitude_t *c) {
  if (!valid(c))
    return;
  if (c->state != IMU_ATTITUDE_RUNNING)
    restart_calibration(c);
  c->gap_pending = true;
}
/* External only: borrow a read-only estimator view; there is no snapshot/lock.
 */
const kalman_attitude_t *imu_attitude_filter(const imu_attitude_t *c) {
  return valid(c) ? &c->filter : NULL;
}
/* External only: reset the filter transactionally, then bypass calibration and
 * discard dynamics history. Future dt is relative to the supplied timestamp. */
kalman_status_t imu_attitude_seed(imu_attitude_t *c, const kalman_real_t q[4],
                                  const kalman_real_t bias[3],
                                  int64_t time_us) {
  if (!valid(c))
    return KALMAN_ERROR_NOT_INITIALIZED;
  if (time_us < 0)
    return KALMAN_ERROR_INVALID_ARGUMENT;
  kalman_status_t status = kalman_attitude_reset(&c->filter, q, bias);
  if (status != KALMAN_OK)
    return status;
  c->state = IMU_ATTITUDE_RUNNING;
  c->previous_us = time_us;
  c->have_time = true;
  c->gap_pending = false;
  memset(&c->dynamics, 0, sizeof(c->dynamics));
  return KALMAN_OK;
}
/* Internal, called by imu_attitude_update after successful gyro prediction.
 * Innovation is measured before all corrections, including jerk rejection.
 * It is Euclidean normalized-direction residual squared, not Mahalanobis NIS.
 */
static void innovation(const kalman_attitude_t *filter, const float accel[3],
                       imu_attitude_result_t *r) {
  const float norm =
      sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
  if (!isfinite(norm) || norm <= 0)
    return;
  const kalman_real_t *q = filter->quaternion;
  const kalman_real_t dx = accel[0] / norm - 2 * (q[1] * q[3] - q[0] * q[2]);
  const kalman_real_t dy = accel[1] / norm - 2 * (q[2] * q[3] + q[0] * q[1]);
  const kalman_real_t dz =
      accel[2] / norm - (1 - 2 * (q[1] * q[1] + q[2] * q[2]));
  r->innovation_squared = dx * dx + dy * dy + dz * dz;
  r->innovation_valid = isfinite(r->innovation_squared);
}
/* External only: validate, consume time, advance startup or predict/correct.
 * Result is per-call; lower-filter diagnostic fields may remain stale when
 * the policy skips its correction. Numeric errors do not roll back this call.
 */
kalman_status_t imu_attitude_update(imu_attitude_t *c,
                                    const imu_attitude_sample_t *s,
                                    imu_attitude_result_t *r) {
  if (!r)
    return KALMAN_ERROR_INVALID_ARGUMENT;
  memset(r, 0, sizeof(*r));
  if (!valid(c))
    return KALMAN_ERROR_NOT_INITIALIZED;
  r->state = c->state;
  if (!s || s->time_us < 0 || (c->have_time && s->time_us <= c->previous_us))
    return KALMAN_ERROR_INVALID_ARGUMENT;
  for (unsigned i = 0; i < 3; ++i)
    if (!isfinite(s->accel_mps2[i]) || !isfinite(s->gyro_rads[i]))
      return KALMAN_ERROR_INVALID_ARGUMENT;
  /* Commit the input timeline only after structural/finite input validation.
   * Even a later skipped sample or numerical failure consumes this timestamp.
   */
  r->dt_us = c->have_time ? s->time_us - c->previous_us : 0;
  r->saturated = s->saturated;
  r->discontinuity =
      s->discontinuity || c->gap_pending ||
      (c->have_time && r->dt_us > (c->state == IMU_ATTITUDE_RUNNING
                                       ? c->config.dynamics_gap_us
                                       : c->config.calibration_gap_us));
  c->previous_us = s->time_us;
  c->have_time = true;
  c->gap_pending = false;
  /* Startup requires one uninterrupted candidate window. A failed heuristic
   * returns OK with CALIBRATION_REJECTED; a failed initializer returns error.
   */
  if (c->state != IMU_ATTITUDE_RUNNING) {
    if (r->discontinuity)
      restart_calibration(c);
    if (s->saturated && c->config.skip_saturated) {
      restart_calibration(c);
      r->event = IMU_ATTITUDE_SKIPPED;
    } else if (c->warmup_remaining) {
      --c->warmup_remaining;
      if (!c->warmup_remaining)
        c->state = IMU_ATTITUDE_CALIBRATING;
    } else {
      calibration_add(&c->calibration, s);
      if (c->calibration.count >= c->config.calibration_samples) {
        kalman_real_t *v = r->calibration_values;
        bool stationary = calibration_is_stationary(
            &c->calibration, &c->config, c->config.filter.gravity_mps2, &v[0],
            &v[1], &v[2], &v[3]);
        kalman_real_t variance[3];
        kalman_status_t status = KALMAN_OK;
        if (stationary) {
          calibration_get_gyro_bias_variance(&c->calibration, &c->config,
                                             variance);
          status = kalman_attitude_initialize_stationary_with_variance(
              &c->filter, c->calibration.accel_mean, c->calibration.gyro_mean,
              variance);
        }
        if (stationary && status == KALMAN_OK) {
          c->state = IMU_ATTITUDE_RUNNING;
          r->event = IMU_ATTITUDE_CALIBRATED;
          memset(&c->dynamics, 0, sizeof(c->dynamics));
          for (unsigned i = 0; i < 3; ++i) {
            r->bias[i] = c->calibration.gyro_mean[i];
            r->bias_std[i] = KALMAN_REAL_SQRT(variance[i]);
          }
        } else {
          restart_calibration(c);
          r->event = IMU_ATTITUDE_CALIBRATION_REJECTED;
        }
        r->state = c->state;
        return status;
      }
    }
    r->state = c->state;
    return KALMAN_OK;
  }
  /* Explicit hold policy: preserve estimate/covariance, rebase time, forget
   * derivative history. This does not reconstruct uncertainty/missing motion.
   */
  if ((r->discontinuity && !c->config.integrate_across_gaps) ||
      (s->saturated && c->config.skip_saturated)) {
    memset(&c->dynamics, 0, sizeof(c->dynamics));
    r->event = IMU_ATTITUDE_SKIPPED;
    return KALMAN_OK;
  }
  /* Predict with gyro first. Dynamics weighting and both filter gates only
   * control accelerometer correction, never undo the successful prediction. */
  bool reject = false;
  float multiplier = 1.0f;
  if (c->config.jerk_enabled)
    multiplier =
        accel_dynamics_quality(&c->dynamics, &c->config, s->accel_mps2,
                               s->time_us, r->dt_us, &reject, &r->jerk_mps3);
  kalman_real_t gyro[3], accel[3];
  for (unsigned i = 0; i < 3; ++i) {
    gyro[i] = s->gyro_rads[i];
    accel[i] = s->accel_mps2[i];
  }
  kalman_status_t status = kalman_attitude_predict_gyro(
      &c->filter, gyro, (kalman_real_t)r->dt_us * (kalman_real_t)1.0e-6f);
  r->event = IMU_ATTITUDE_UPDATED;
  if (status != KALMAN_OK)
    return status;
  innovation(&c->filter, s->accel_mps2, r);
  if (reject) {
    r->correction = IMU_ATTITUDE_REJECT_JERK;
    return KALMAN_OK;
  }
  r->dynamic_downweighted = multiplier > 1.0f;
  status = kalman_attitude_correct_accel_with_variance(
      &c->filter, accel,
      c->config.filter.accel_direction_variance * (kalman_real_t)multiplier);
  if (status != KALMAN_OK)
    return status;
  if (c->filter.last_accel_correction_applied) {
    r->correction = IMU_ATTITUDE_CORRECTION_USED;
    r->effective_accel_variance = c->filter.last_accel_measurement_variance;
  } else {
    kalman_real_t error =
        c->filter.last_accel_norm_mps2 - c->config.filter.gravity_mps2;
    if (error < 0)
      error = -error;
    r->correction = error > c->config.filter.accel_rejection_threshold_mps2
                        ? IMU_ATTITUDE_REJECT_MAGNITUDE
                        : IMU_ATTITUDE_REJECT_INNOVATION;
  }
  return KALMAN_OK;
}
