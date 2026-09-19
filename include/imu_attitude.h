#pragma once
#include "kalman_attitude.h"
#ifdef __cplusplus
extern "C" {
#endif

/** Maximum moving-average window in samples; storage is caller-owned. */
#define IMU_ATTITUDE_MAX_AVERAGE 20U
/** Maximum lag between averaged acceleration samples; not a duration. */
#define IMU_ATTITUDE_MAX_DIFFERENCE 10U
/** Startup state after the most recently consumed input. */
typedef enum {
  IMU_ATTITUDE_WARMUP, /**< Discard valid samples before estimating bias. */
  IMU_ATTITUDE_CALIBRATING, /**< Accumulate a candidate stationary window. */
  IMU_ATTITUDE_RUNNING /**< Alignment available; does not certify accuracy. */
} imu_attitude_state_t;
/** Per-call event; always inspect the status code before using this value. */
typedef enum {
  IMU_ATTITUDE_NO_EVENT,   /**< Warmup or incomplete calibration window. */
  IMU_ATTITUDE_CALIBRATED, /**< Stationary initialization succeeded. */
  IMU_ATTITUDE_CALIBRATION_REJECTED, /**< Retry starts with warmup. */
  IMU_ATTITUDE_UPDATED, /**< Prediction attempted; check return status. */
  IMU_ATTITUDE_SKIPPED  /**< Time rebased without a new attitude estimate. */
} imu_attitude_event_t;
/** Acceleration correction decision for this call, not the last filter call. */
typedef enum {
  IMU_ATTITUDE_CORRECTION_NONE, /**< No completed correction decision. */
  IMU_ATTITUDE_CORRECTION_USED, /**< Measurement applied successfully. */
  IMU_ATTITUDE_REJECT_JERK, /**< Policy gate skipped the filter correction. */
  IMU_ATTITUDE_REJECT_MAGNITUDE, /**< Norm differs too much from gravity. */
  IMU_ATTITUDE_REJECT_INNOVATION /**< Normalized direction gate rejected it. */
} imu_attitude_correction_t;

/** Per-instance configuration, copied by init; no retained pointers. All real
 * parameters must be finite. Window validation also applies with jerk disabled.
 * Do not change ctx.config directly; initialize again to change configuration.
 */
typedef struct {
  kalman_attitude_config_t filter;  /**< Mathematical filter configuration. */
  uint32_t warmup_samples;          /**< Discard count; zero bypasses warmup. */
  uint32_t calibration_samples;     /**< Accepted window size: 2..1000000. */
  kalman_real_t max_accel_std_mps2; /**< Maximum axis std, m/s^2, >=0. */
  kalman_real_t max_gyro_std_rads;  /**< Maximum axis std, rad/s, >=0. */
  kalman_real_t max_gyro_mean_rads; /**< Maximum gyro mean norm, rad/s, >=0. */
  kalman_real_t
      gravity_tolerance_mps2; /**< |mean accel norm - g| limit, >=0. */
  kalman_real_t
      bias_std_floor_rads; /**< Bias std floor >=0; square must fit. */
  bool jerk_enabled; /**< Enable moving-average acceleration dynamics gate. */
  uint32_t average_samples; /**< Moving average length: 1..20 samples. */
  uint32_t
      difference_samples; /**< Average differentiation lag: 1..10 samples. */
  float jerk_soft_mps3;   /**< Downweight above this jerk norm, >=0, m/s^3. */
  float jerk_hard_mps3;   /**< Reject at/above this norm; >soft, m/s^3. */
  float jerk_max_variance_multiplier; /**< Quadratic weighting ceiling, >=1. */
  uint32_t calibration_gap_us; /**< Positive startup inter-sample gap limit. */
  uint32_t dynamics_gap_us;    /**< Positive running gap/history-reset limit. */
  /* Default true preserves the old example: use elapsed time and current gyro
   * over gaps. This cannot reconstruct missing motion. false skips the first
   * sample after a reported gap or dt>dynamics_gap_us, preserving orientation.
   */
  bool integrate_across_gaps;
  bool skip_saturated; /* Default false preserves old diagnostic-only behavior.
                        */
} imu_attitude_config_t;

/** One acquired sample; copied/consumed synchronously, never retained. */
typedef struct {
  float accel_mps2[3]; /**< Specific force in common body XYZ axes, m/s^2. */
  float gyro_rads[3];  /**< Angular velocity in the same axes, rad/s. */
  int64_t time_us; /* Nonnegative, strictly increasing accepted timestamps. */
  bool
      discontinuity; /**< Acquisition detected missing/unreliable continuity. */
  bool saturated;    /**< Acquisition detected saturation; not inferred here. */
} imu_attitude_sample_t;

/** Caller-owned output, cleared at entry. Unavailable quantities remain zero;
 * zero alone is not a validity indicator. Must not overlap context or sample.
 */
typedef struct {
  imu_attitude_state_t
      state; /**< State after processing; meaningful if initialized. */
  imu_attitude_event_t event; /**< Startup/processing event for this call. */
  imu_attitude_correction_t correction; /**< This call's correction outcome. */
  bool discontinuity; /**< Sample flag, pending notification or excessive dt. */
  bool saturated;     /**< Copy of the valid input's saturation flag. */
  bool innovation_valid; /**< innovation_squared was computed and is finite. */
  bool dynamic_downweighted; /**< Jerk multiplier >1 submitted to correction. */
  int64_t dt_us;   /**< Elapsed time since previous consumed input; first=0. */
  float jerk_mps3; /**< Smoothed body-frame derivative norm; unavailable=0. */
  kalman_real_t
      innovation_squared; /**< Dimensionless direction residual norm^2. */
  kalman_real_t
      effective_accel_variance;        /**< Dimensionless R, only when USED. */
  kalman_real_t calibration_values[4]; /**< Calibration events: |mean a| m/s^2,
      |mean gyro| rad/s, maximum axis accel std m/s^2, gyro std rad/s. */
  kalman_real_t bias[3];     /**< CALIBRATED only: initial XYZ bias, rad/s. */
  kalman_real_t bias_std[3]; /**< CALIBRATED only: initial XYZ std, rad/s. */
} imu_attitude_result_t;

/* Public layout permits static allocation. Internal fields: do not mutate. */
typedef struct {
  kalman_real_t accel_mean[3]; /**< Internal running mean, m/s^2. */
  kalman_real_t accel_m2[3];   /**< Internal sum of squared accel deviations. */
  kalman_real_t gyro_mean[3];  /**< Internal running mean, rad/s. */
  kalman_real_t gyro_m2[3];    /**< Internal sum of squared gyro deviations. */
  uint32_t count;              /**< Internal number of accumulated samples. */
} imu_attitude_calibration_t;
/** Internal fixed-capacity jerk history; exposed only for static allocation. */
typedef struct {
  float window[IMU_ATTITUDE_MAX_AVERAGE][3]; /**< Acceleration ring, m/s^2. */
  float sum[3]; /**< Rolling sum of populated window entries, m/s^2. */
  float mean_history[IMU_ATTITUDE_MAX_DIFFERENCE][3]; /**< Mean ring, m/s^2. */
  int64_t
      mean_time_us[IMU_ATTITUDE_MAX_DIFFERENCE]; /**< Mean timestamps, us. */
  uint32_t window_count; /**< Populated acceleration entries. */
  uint32_t window_index; /**< Next acceleration ring slot. */
  uint32_t mean_count;   /**< Populated mean entries. */
  uint32_t mean_index;   /**< Next mean ring slot. */
} imu_attitude_dynamics_t;
/** Caller-owned context, no heap resources/destructor. Do not mutate fields.
 * Keep alive for every API call and serialize all access, including getters. */
typedef struct {
  imu_attitude_config_t config;           /**< Copied initialization policy. */
  kalman_attitude_t filter;               /**< Owned mathematical estimator. */
  imu_attitude_state_t state;             /**< Startup/operating state. */
  imu_attitude_calibration_t calibration; /**< Stationarity accumulators. */
  imu_attitude_dynamics_t dynamics;       /**< Jerk history. */
  uint32_t warmup_remaining;              /**< Samples still to discard. */
  uint32_t tag; /**< Internal initialization marker, not memory validation. */
  int64_t previous_us; /**< Most recently consumed input timestamp. */
  bool have_time;      /**< previous_us is initialized. */
  bool gap_pending;    /**< Gap to report with next valid input. */
} imu_attitude_t;

/** @name Public API contracts
 * All six functions are external entry points; none is called internally by
 * this component. Inputs/outputs are borrowed for the call, except the const
 * filter view. Use distinct, nonoverlapping input/output/context storage;
 * init explicitly supports config == &ctx->config. No API performs I/O,
 * allocation, locking or retries. These are task/main-loop APIs, not ISR APIs.
 * Initialize context before use; zero-initialize it if probing initialization.
 */

/** Default profile reproduces the MP65 example at nominal 1 kHz: 500 discarded
 * samples, 2000 calibration samples, 20/10 sample jerk windows, 15/60 m/s^3.
 * Thresholds are heuristics, not a guarantee of stationarity or accuracy. */
imu_attitude_config_t imu_attitude_config_default(void);
/** Initialize caller-owned context. No allocation, I/O, OS or locking.
 * ctx and config must be non-NULL. Returns KALMAN_OK, INVALID_ARGUMENT for
 * invalid policy, or the filter initializer's error. Configuration is copied.
 * Invalid configuration leaves context unchanged. One owner per context;
 * serialize calls. All components must use the same kalman_real_t ABI. */
kalman_status_t imu_attitude_init(imu_attitude_t *ctx,
                                  const imu_attitude_config_t *config);
/** Process one sample. Result is reset each call. Invalid/nonmonotonic input
 * leaves context unchanged. A numerical filter error may leave a successful
 * gyro prediction applied; inspect status before consuming result. No retries.
 * ctx must be initialized; sample/result must be non-NULL. Returns OK also
 * for heuristic rejection or intentional skip, NOT_INITIALIZED for invalid
 * context, INVALID_ARGUMENT for invalid input, or the filter's error.
 * Finite but extreme values can overflow intermediates: supply physical SI
 * sensor ranges, not arbitrary finite floats. Time/history can advance on a
 * numerical failure; retrying the same timestamp is not supported.
 * Bounded work, fixed storage; never called from a hardware ISR by default. */
kalman_status_t imu_attitude_update(imu_attitude_t *ctx,
                                    const imu_attitude_sample_t *sample,
                                    imu_attitude_result_t *result);
/** Notify missing acquisition without fabricating a sample. Before alignment,
 * restart warmup/calibration; while running mark a pending gap. No filter
 * reset. Timestamp continuity is evaluated again on the next valid sample.
 * NULL/uninitialized context is a no-op; there is no return status. */
void imu_attitude_notify_gap(imu_attitude_t *ctx);
/** Explicit initial orientation/bias for applications unable to start at rest.
 * Uses filter.config initial covariance, bypasses stationary calibration.
 * Invalid seed leaves context unchanged; next sample must have later time.
 * quaternion: finite nonzero [w,x,y,z], normalized by reset; bias_rads:
 * three finite values (both arrays are required). time_us must be nonnegative.
 * Returns OK, NOT_INITIALIZED, INVALID_ARGUMENT or reset's numerical error. */
kalman_status_t imu_attitude_seed(imu_attitude_t *ctx,
                                  const kalman_real_t quaternion[4],
                                  const kalman_real_t bias_rads[3],
                                  int64_t time_us);
/** Read-only filter view for existing getters; lifetime is the context's.
 * NULL for an uninitialized context. No concurrent access during updates. */
const kalman_attitude_t *imu_attitude_filter(const imu_attitude_t *ctx);
#ifdef __cplusplus
}
#endif
