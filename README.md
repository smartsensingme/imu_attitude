# imu_attitude

Portable C policy layer above `kalman_attitude`. It owns startup calibration,
dynamic acceleration weighting and sequencing of the 6-DoF estimator. It knows
nothing about MPU registers, SPI, GPIO, FreeRTOS, USB or rendering. It does not
replace the mathematical filter, and applications may still use that filter
directly. This repository is independent of the sensor-driver application and
can be used from ESP-IDF or another C99 build.

## Use from GitHub or a native C project

The released component is available at
[`https://github.com/smartsensingme/imu_attitude.git`](https://github.com/smartsensingme/imu_attitude.git).
For an ESP-IDF application using the Component Manager, declare only the direct
dependency in the consuming component's `main/idf_component.yml`:

```yaml
dependencies:
  imu_attitude:
    git: https://github.com/smartsensingme/imu_attitude.git
    version: "0.1.0"
```

The component manifest resolves the required
[`kalman_attitude`](https://github.com/smartsensingme/kalman_attitude.git)
dependency, which in turn resolves the base
[`kalman`](https://github.com/smartsensingme/kalman.git) component. For an
offline/manual installation, clone all three HTTPS repositories into the
component search path, or provide the sibling source paths to native CMake.
No developer-specific SSH host alias is part of the manifests or documentation.

## Responsibilities

The driver supplies accel in m/s², gyro in rad/s, monotonic microsecond sample
timestamps and optional discontinuity/saturation flags. Both vectors must use
the same body coordinate axes. `imu_attitude` discards a warmup window, collects
Welford means/variances, checks stationarity heuristics, initializes attitude
and gyro bias, then sequences gyro prediction and gated accel correction.
Quaternion convention and gravity reference are those of `kalman_attitude`.
No heading reference is created: yaw remains relative.

The application retains acquisition timing, GPIO/INT, core/priority selection,
hardware failure detection, deadline recovery and output routing. Notify a
missing acquisition without inventing a sample. A reported gap cannot recover
motion that was never measured. Scheduling diagnostics and raw ADC saturation
thresholds stay outside this library; fusion quality diagnostics are returned.

## Integration

For ESP-IDF, add `imu_attitude` to `PRIV_REQUIRES`, with sibling components
`kalman_attitude` and `kalman` available. There are no Kconfig options here:
configuration is per context. For other C99 environments, compile
`imu_attitude.c` with the two portable dependencies and the required math library.
All translation units must agree on the Kalman numeric ABI. Physical inputs and
the jerk window use float; estimator arithmetic uses `kalman_real_t`.

```c
#include "imu_attitude.h"
static imu_attitude_t attitude; /* Caller-owned, no dynamic allocation. */

kalman_status_t setup(void) {
    imu_attitude_config_t config = imu_attitude_config_default();
    /* Adjust sample-count windows and gap limits when changing acquisition rate. */
    /* Caller must stop setup if this is not KALMAN_OK. */
    return imu_attitude_init(&attitude, &config);
}

kalman_status_t process_sample(const imu_attitude_sample_t *sample) {
    imu_attitude_result_t result;
    kalman_status_t status = imu_attitude_update(&attitude, sample, &result);
    if (status != KALMAN_OK) {
        return status; /* Apply an application-owned fault policy; no blind retry. */
    }
    if (result.event == IMU_ATTITUDE_CALIBRATED ||
        result.event == IMU_ATTITUDE_UPDATED) {
        kalman_real_t q[4];
        status = kalman_attitude_get_quaternion(imu_attitude_filter(&attitude), q);
        if (status != KALMAN_OK) return status;
        /* Copy q/result to an application-owned output path. */
    }
    return KALMAN_OK; /* Warmup/rejection/skip are valid policy outcomes. */
}
```

One owner per context, no internal synchronization. Serialize init/update/seed/
notification and read access. `imu_attitude_filter()` returns a const view for
existing getters, not permission to mutate estimator internals. Context layout
is public solely for static/stack allocation. Work is bounded by fixed window
capacity, 3-axis loops and the existing fixed-size estimator. No allocation,
logging, clock reads or I/O in update. Time comes from the caller.

## States and defaults

`WARMUP -> CALIBRATING -> RUNNING`; rejected calibration restarts warmup.
Warmup=0 begins directly in CALIBRATING. Defaults reproduce the former MP65
example at nominal 1 kHz:

| Configuration | Default |
|---|---|
| Warmup / calibration | 500 / 2000 valid samples |
| Max accel / gyro standard deviation | 0.15 m/s² / 0.01 rad/s |
| Max gyro mean norm | 0.15 rad/s |
| Gravity norm tolerance | 0.5 m/s² |
| Gyro bias uncertainty floor | 0.000872664626 rad/s (0.05 deg/s) |
| Jerk average / difference | 20 / 10 samples |
| Jerk soft / hard | 15 / 60 m/s³ |
| Maximum jerk variance multiplier | 25 |
| Calibration / dynamics gap | 1500 / 2000 µs |
| Integrate across gaps / skip saturated | true / false |

Calibration uncertainty is sample variance divided by count, bounded below by
the configured squared bias floor. With correlated samples this is not a fully
validated statistical confidence interval. Stationarity tests are heuristics:
they cannot guarantee absolute rest or perfect bias calibration.

Windows are sample counts, NOT automatically rescaled durations. Runtime jerk
windows may be 1..20 and 1..10; calibration 2..1,000,000 samples. Configure gaps
and windows explicitly for a different output rate. `jerk_enabled=false`
bypasses the dynamic gate while retaining the mathematical filter's gates.
Body-frame jerk includes changing gravity under rotation, not just translation.

`imu_attitude_seed()` bypasses stationary initialization with a supplied
quaternion, bias and timestamp. Initial covariance comes from filter config.
This supports starting with externally supplied calibration; it does not itself
estimate bias during motion. Reinitializing the context restarts calibration.

## Gaps, errors and result interpretation

`imu_attitude_notify_gap()` restarts pre-alignment calibration or marks a pending
gap while running. Sample flags and timing also detect discontinuity. Calibration
never continues a window across a detected gap. In RUNNING, defaults preserve
the old behavior: predict with the current gyro over elapsed time; reset the
jerk history when dt exceeds the dynamics gap. A reported gap alone does not
reset the orientation or gyro bias. This approximation is not loss recovery.
Set `integrate_across_gaps=false` to skip the first sample after a gap and rebase
time; orientation/covariance are held, so motion and uncertainty over that gap
are still unmodelled. `skip_saturated=true` similarly skips saturated input
(or restarts calibration). Default false keeps the previous diagnostic-only
behavior. The application must interpret a SKIPPED result as not newly estimated.

Update clears its result each call. It reports state, calibration event, dt,
jerk, pre-correction normalized-direction innovation squared and correction
reason: used, jerk rejection, magnitude rejection or innovation rejection.
`dynamic_downweighted` means a jerk-inflated variance was submitted; it can
coexist with subsequent rejection by another gate. `effective_accel_variance`
is populated for applied corrections and includes the filter's innovation
weighting. Calibration diagnostics are meaningful on completion/rejection;
bias and bias_std are populated only on CALIBRATED. Innovation may be valid
even when correction is rejected by jerk.

NaN/Inf inputs and non-increasing/negative timestamps are rejected before
context changes. A filter numerical failure can occur after a successful gyro
prediction; update is not an atomic rollback. Check the return code, not only
RUNNING state. Startup/calibration state is not a guarantee of future accuracy.

Finite but extreme inputs can still overflow intermediate products. Supply
physical sensor ranges, not arbitrary finite floats. A numeric error may have
consumed the timestamp and updated dynamics history; do not retry that same
timestamp. `UPDATED` means prediction was attempted, not necessarily successful.

## API and ownership reference

All six public functions are external entry points (none calls another public
`imu_attitude_*` function internally). Contexts own all state inline; no destroy
operation is needed. Inputs are borrowed only for the call. Do not overlap
sample/result/context storage. Initialization explicitly permits configuration
from `&ctx.config`; it copies before clearing context. ABI layout is not a
serialized format: rebuild all consumers when headers/numeric type change.

| API | Contract |
|---|---|
| `config_default()` | Returns the nominal 1 kHz policy by value, no side effects. |
| `init(ctx, config)` | Both required. Returns OK or invalid-policy/filter error; leaves ctx unchanged on error. Copies config and starts warmup/calibration. |
| `update(ctx, sample, result)` | Required initialized ctx and non-NULL buffers. Clears result first. Returns OK also for rejection/skip; check event and correction. |
| `notify_gap(ctx)` | NULL/uninitialized is a no-op. Startup restarts; running filter is retained with pending discontinuity. |
| `seed(ctx, q, bias, time_us)` | Required finite q[4]/bias[3], q nonzero, time >=0. Resets filter and jerk history, enters RUNNING. Failure preserves context. |
| `filter(ctx)` | Borrowed const pointer until context ends/reinitializes; NULL for NULL/uninitialized context. No concurrent update while reading. |

`update` and `seed` return `KALMAN_ERROR_NOT_INITIALIZED` for NULL or
uninitialized contexts, `KALMAN_ERROR_INVALID_ARGUMENT` for invalid input, or
propagate the mathematical filter's error. Use a zero-initialized context if
probing its initialization status. Get current bias through the filter getter;
the result's bias array is not continuously populated.

Troubleshooting: repeated calibration rejection means checking rest, units,
axes, sample gaps and thresholds before loosening gates. Persistent jerk
rejection may be physical vibration or rotation of gravity; these thresholds
are not sensor-independent guarantees. Stable but offset tilt needs physical
alignment/calibration checks. Yaw drift is not repaired by acceleration gating.

## Extraction and validation

The normal-path numerical algorithm and defaults were retained. One intentional
numerical improvement: jerk differentiation uses differences of int64 timestamps
instead of an ever-growing float time accumulator. This avoids degrading short
interval precision on long runs; it means results are not promised bit-identical.
Additional explicit invalid-input, gap and saturation policies are tested.

```sh
cmake -S . -B /tmp/imu-attitude-tests \
  -DIMU_ATTITUDE_KALMAN_ATTITUDE_SOURCE_DIR=/path/to/kalman_attitude
cmake --build /tmp/imu-attitude-tests
ctest --test-dir /tmp/imu-attitude-tests --output-on-failure
```

Tests compare against frozen pre-extraction calibration/jerk code and direct
filter calls on a deterministic synthetic trajectory, with 2e-4 absolute
tolerance on quaternion/covariance elements, not a physical accuracy claim.
They also cover calibration rejection, external seed, gaps, saturation,
invalid input/configuration, magnitude and innovation rejection. Hardware
timing/stack headroom and physical roll/pitch/yaw accuracy must be revalidated
after integrating this layer. Neither these tests nor the extraction resolve
the previously observed yaw return errors.
