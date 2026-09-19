#include "imu_attitude.h"
#include "legacy_reference.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void regression(void) {
  imu_attitude_t c;
  imu_attitude_config_t cfg = imu_attitude_config_default();
  assert(imu_attitude_init(&c, &cfg) == KALMAN_OK);
  stationary_calibration_t cal;
  calibration_reset(&cal);
  imu_attitude_sample_t s = {.accel_mps2 = {0, 0, 9.80665f},
                             .gyro_rads = {.01f, -.02f, .03f}};
  imu_attitude_result_t r;
  for (unsigned i = 1; i <= 2500; ++i) {
    s.time_us = i * 1000;
    assert(imu_attitude_update(&c, &s, &r) == KALMAN_OK);
    if (i > 500)
      calibration_add(&cal, &s);
    if (i < 2500)
      assert(r.event != IMU_ATTITUDE_CALIBRATED);
  }
  assert(r.event == IMU_ATTITUDE_CALIBRATED && c.state == IMU_ATTITUDE_RUNNING);
  kalman_real_t vals[4], variance[3];
  assert(calibration_is_stationary(&cal, cfg.filter.gravity_mps2, &vals[0],
                                   &vals[1], &vals[2], &vals[3]));
  calibration_get_gyro_bias_variance(&cal, variance);
  kalman_attitude_t reference;
  assert(kalman_attitude_init(&reference, &cfg.filter) == KALMAN_OK);
  assert(kalman_attitude_initialize_stationary_with_variance(
             &reference, cal.accel_mean, cal.gyro_mean, variance) == KALMAN_OK);
  accel_dynamics_t dynamics = {0};
  unsigned rejected = 0, attenuated = 0;
  for (unsigned i = 0; i < 2000; ++i) {
    s.time_us += 1000;
    s.accel_mps2[0] =
        (i >= 500 && i < 550) ? 3.0f : 0.05f * sinf((float)i * .03f);
    s.gyro_rads[0] = .01f + .05f * sinf((float)i * .01f);
    bool reject;
    float jerk;
    float multiplier =
        accel_dynamics_quality(&dynamics, s.accel_mps2, .001f, &reject, &jerk);
    assert(kalman_attitude_predict_gyro(&reference, s.gyro_rads, .001f) ==
           KALMAN_OK);
    if (!reject)
      assert(kalman_attitude_correct_accel_with_variance(
                 &reference, s.accel_mps2,
                 cfg.filter.accel_direction_variance * multiplier) ==
             KALMAN_OK);
    assert(imu_attitude_update(&c, &s, &r) == KALMAN_OK);
    assert((r.correction == IMU_ATTITUDE_REJECT_JERK) == reject);
    rejected += reject;
    attenuated += r.dynamic_downweighted;
    for (unsigned j = 0; j < 4; ++j)
      assert(fabsf(c.filter.quaternion[j] - reference.quaternion[j]) < 2e-4f);
    for (unsigned j = 0; j < 36; ++j)
      assert(fabsf(c.filter.covariance[j] - reference.covariance[j]) < 2e-4f);
    for (unsigned j = 0; j < 3; ++j)
      assert(c.filter.gyro_bias_rads[j] == cal.gyro_mean[j]);
  }
  assert(rejected > 0);
  (void)attenuated;
}

static void edge_cases(void) {
  imu_attitude_t c = {0}, saved;
  imu_attitude_config_t cfg = imu_attitude_config_default();
  cfg.warmup_samples = 0;
  cfg.calibration_samples = 2;
  imu_attitude_sample_t s = {
      .accel_mps2 = {0, 0, 9.80665f}, .gyro_rads = {1, 0, 0}, .time_us = 1000};
  imu_attitude_result_t r;
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_ERROR_NOT_INITIALIZED);
  assert(imu_attitude_init(&c, &cfg) == KALMAN_OK);
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_OK);
  s.time_us += 1000;
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_OK);
  assert(r.event == IMU_ATTITUDE_CALIBRATION_REJECTED);
  saved = c;
  cfg.average_samples = 0;
  assert(imu_attitude_init(&c, &cfg) == KALMAN_ERROR_INVALID_ARGUMENT);
  assert(!memcmp(&c, &saved, sizeof(c)));
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_ERROR_INVALID_ARGUMENT);
  assert(!memcmp(&c, &saved, sizeof(c)));
  s.time_us++;
  s.accel_mps2[0] = NAN;
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_ERROR_INVALID_ARGUMENT);
  assert(!memcmp(&c, &saved, sizeof(c)));
  cfg = imu_attitude_config_default();
  cfg.integrate_across_gaps = false;
  cfg.skip_saturated = true;
  cfg.jerk_enabled = false;
  assert(imu_attitude_init(&c, &cfg) == KALMAN_OK);
  kalman_real_t q[4] = {1, 0, 0, 0}, bias[3] = {0};
  assert(imu_attitude_seed(&c, q, bias, 1000000) == KALMAN_OK);
  s = (imu_attitude_sample_t){.time_us = 1001000,
                              .accel_mps2 = {0, 0, 9.80665f}};
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_OK);
  assert(r.correction == IMU_ATTITUDE_CORRECTION_USED);
  imu_attitude_notify_gap(&c);
  s.time_us += 1000;
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_OK);
  assert(r.event == IMU_ATTITUDE_SKIPPED && r.discontinuity);
  s.time_us += 1000;
  s.saturated = true;
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_OK &&
         r.event == IMU_ATTITUDE_SKIPPED);
  s.saturated = false;
  s.time_us += 1000;
  s.accel_mps2[2] = 20;
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_OK &&
         r.correction == IMU_ATTITUDE_REJECT_MAGNITUDE);
  s.time_us += 1000;
  s.accel_mps2[2] = 0;
  s.accel_mps2[0] = 9.80665f;
  assert(imu_attitude_update(&c, &s, &r) == KALMAN_OK &&
         r.correction == IMU_ATTITUDE_REJECT_INNOVATION);
  assert(imu_attitude_init(&c, &cfg) == KALMAN_OK);
  imu_attitude_notify_gap(&c);
  assert(c.warmup_remaining == 500 && c.calibration.count == 0);
}
static void time_and_windows(void) {
  imu_attitude_config_t cfg = imu_attitude_config_default();
  cfg.average_samples = 1;
  cfg.difference_samples = 1;
  imu_attitude_t early, late;
  assert(imu_attitude_init(&early, &cfg) == KALMAN_OK);
  assert(imu_attitude_init(&late, &cfg) == KALMAN_OK);
  kalman_real_t q[4] = {1, 0, 0, 0}, bias[3] = {0};
  const int64_t origin = INT64_C(9000000000000);
  assert(imu_attitude_seed(&early, q, bias, 0) == KALMAN_OK);
  assert(imu_attitude_seed(&late, q, bias, origin) == KALMAN_OK);
  imu_attitude_result_t a, b;
  for (int i = 1; i <= 3; ++i) {
    imu_attitude_sample_t sample = {.time_us = i * 1000,
                                    .accel_mps2 = {0, 0, 9.80665f}};
    sample.accel_mps2[0] = i == 1 ? 0.0f : (i == 2 ? 0.03f : 0.2f);
    assert(imu_attitude_update(&early, &sample, &a) == KALMAN_OK);
    sample.time_us += origin;
    assert(imu_attitude_update(&late, &sample, &b) == KALMAN_OK);
    assert(a.jerk_mps3 == b.jerk_mps3);
    assert(!memcmp(early.filter.quaternion, late.filter.quaternion,
                   sizeof(early.filter.quaternion)));
    if (i == 2) assert(a.dynamic_downweighted && a.jerk_mps3 > 15);
    if (i == 3) assert(a.correction == IMU_ATTITUDE_REJECT_JERK);
  }
  cfg.warmup_samples = 1;
  cfg.calibration_samples = 2;
  assert(imu_attitude_init(&early, &cfg) == KALMAN_OK);
  imu_attitude_sample_t sample = {.time_us = 1000, .accel_mps2 = {0, 0, 9.80665f}};
  assert(imu_attitude_update(&early, &sample, &a) == KALMAN_OK);
  sample.time_us += 1000;
  assert(imu_attitude_update(&early, &sample, &a) == KALMAN_OK);
  assert(early.calibration.count == 1);
  sample.time_us += 3000;
  assert(imu_attitude_update(&early, &sample, &a) == KALMAN_OK);
  assert(a.discontinuity && early.calibration.count == 0);
}
int main(void) {
  regression();
  edge_cases();
  time_and_windows();
  puts("imu_attitude tests passed");
}
