/* Frozen pre-extraction policy from mp65_attitude/main.c. Regression oracle;
 * do not update together with the production implementation. */
#include <math.h>
#include <string.h>
typedef imu_attitude_sample_t esp_mp65_sample_t;
#define CALIBRATION_WARMUP_SAMPLES (500000U / 1000U)
#define CALIBRATION_SAMPLE_COUNT (2000000U / 1000U)
#define CALIBRATION_MAX_ACCEL_STD_MPS2 0.15f
#define CALIBRATION_MAX_GYRO_STD_RADS 0.01f
#define CALIBRATION_MAX_GYRO_MEAN_RADS 0.15f
#define CALIBRATION_GRAVITY_TOLERANCE_MPS2 0.50f
#define CALIBRATION_GYRO_BIAS_STD_FLOOR_RADS 0.000872664626f
#define ACCEL_JERK_AVERAGE_SAMPLES 20U
#define ACCEL_JERK_DIFFERENCE_SAMPLES 10U
#define ACCEL_JERK_SOFT_MPS3 15.0f
#define ACCEL_JERK_HARD_MPS3 60.0f
#define ACCEL_JERK_MAX_VARIANCE_MULTIPLIER 25.0f
typedef struct {
  float window[ACCEL_JERK_AVERAGE_SAMPLES][3];
  float sum[3];
  float mean_history[ACCEL_JERK_DIFFERENCE_SAMPLES][3];
  float mean_time_history[ACCEL_JERK_DIFFERENCE_SAMPLES];
  float elapsed_seconds;
  uint32_t window_count, window_index;
  uint32_t mean_count, mean_index;
} accel_dynamics_t;

/* The instantaneous 1 ms acceleration difference is noise-amplifying. Average
 * 20 samples first, then differentiate rolling means 10 ms apart. Soft dynamics
 * lower correction influence; a very abrupt change leaves gyro propagation
 * intact but suppresses that gravity correction. */
static float accel_dynamics_quality(accel_dynamics_t *dynamics,
                                    const float accel_mps2[3], float dt_seconds,
                                    bool *reject, float *jerk_mps3) {
  float squared = 0.0f;
  float mean[3];
  *reject = false;
  *jerk_mps3 = 0.0f;
  if (dt_seconds <= 0.0f || dt_seconds > 0.002f) {
    memset(dynamics, 0, sizeof(*dynamics));
  }
  dynamics->elapsed_seconds += dt_seconds;
  for (size_t axis = 0; axis < 3; ++axis) {
    if (dynamics->window_count == ACCEL_JERK_AVERAGE_SAMPLES)
      dynamics->sum[axis] -= dynamics->window[dynamics->window_index][axis];
    dynamics->window[dynamics->window_index][axis] = accel_mps2[axis];
    dynamics->sum[axis] += accel_mps2[axis];
  }
  dynamics->window_index =
      (dynamics->window_index + 1U) % ACCEL_JERK_AVERAGE_SAMPLES;
  if (dynamics->window_count < ACCEL_JERK_AVERAGE_SAMPLES)
    ++dynamics->window_count;
  if (dynamics->window_count != ACCEL_JERK_AVERAGE_SAMPLES)
    return 1.0f;
  for (size_t axis = 0; axis < 3; ++axis)
    mean[axis] = dynamics->sum[axis] / (float)ACCEL_JERK_AVERAGE_SAMPLES;
  if (dynamics->mean_count == ACCEL_JERK_DIFFERENCE_SAMPLES) {
    const float elapsed = dynamics->elapsed_seconds -
                          dynamics->mean_time_history[dynamics->mean_index];
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
  dynamics->mean_time_history[dynamics->mean_index] = dynamics->elapsed_seconds;
  dynamics->mean_index =
      (dynamics->mean_index + 1U) % ACCEL_JERK_DIFFERENCE_SAMPLES;
  if (*jerk_mps3 >= ACCEL_JERK_HARD_MPS3) {
    *reject = true;
    return 1.0f;
  }
  if (*jerk_mps3 <= ACCEL_JERK_SOFT_MPS3) {
    return 1.0f;
  }
  const float ratio = (*jerk_mps3 - ACCEL_JERK_SOFT_MPS3) /
                      (ACCEL_JERK_HARD_MPS3 - ACCEL_JERK_SOFT_MPS3);
  return 1.0f + (ACCEL_JERK_MAX_VARIANCE_MULTIPLIER - 1.0f) * ratio * ratio;
}

typedef struct {
  kalman_real_t accel_mean[3];
  kalman_real_t accel_m2[3];
  kalman_real_t gyro_mean[3];
  kalman_real_t gyro_m2[3];
  uint32_t count;
} stationary_calibration_t;

static void calibration_reset(stationary_calibration_t *calibration) {
  memset(calibration, 0, sizeof(*calibration));
}

/* Welford accumulation avoids cancellation while estimating stationarity. */
static void calibration_add(stationary_calibration_t *calibration,
                            const esp_mp65_sample_t *sample) {
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

static bool calibration_is_stationary(
    const stationary_calibration_t *calibration, kalman_real_t gravity_mps2,
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

  *accel_norm_mps2 = KALMAN_REAL_SQRT(accel_squared);
  *gyro_norm_rads = KALMAN_REAL_SQRT(gyro_squared);
  *max_accel_std_mps2 = KALMAN_REAL_SQRT(max_accel_variance);
  *max_gyro_std_rads = KALMAN_REAL_SQRT(max_gyro_variance);
  gravity_error = *accel_norm_mps2 - gravity_mps2;
  if (gravity_error < (kalman_real_t)0) {
    gravity_error = -gravity_error;
  }

  return gravity_error <= (kalman_real_t)CALIBRATION_GRAVITY_TOLERANCE_MPS2 &&
         *gyro_norm_rads <= (kalman_real_t)CALIBRATION_MAX_GYRO_MEAN_RADS &&
         *max_accel_std_mps2 <= (kalman_real_t)CALIBRATION_MAX_ACCEL_STD_MPS2 &&
         *max_gyro_std_rads <= (kalman_real_t)CALIBRATION_MAX_GYRO_STD_RADS;
}

static void
calibration_get_gyro_bias_variance(const stationary_calibration_t *calibration,
                                   kalman_real_t gyro_bias_variance_rads2[3]) {
  const kalman_real_t sample_divisor = (kalman_real_t)(calibration->count - 1U);
  const kalman_real_t mean_divisor = (kalman_real_t)calibration->count;
  const kalman_real_t variance_floor =
      (kalman_real_t)CALIBRATION_GYRO_BIAS_STD_FLOOR_RADS *
      (kalman_real_t)CALIBRATION_GYRO_BIAS_STD_FLOOR_RADS;
  size_t axis;

  for (axis = 0U; axis < 3U; ++axis) {
    const kalman_real_t sample_variance =
        calibration->gyro_m2[axis] / sample_divisor;
    const kalman_real_t mean_variance = sample_variance / mean_divisor;
    gyro_bias_variance_rads2[axis] =
        mean_variance > variance_floor ? mean_variance : variance_floor;
  }
}
