# ShaftKF PX4 module v19

This package is a PX4 module skeleton for the Shaft Kalman Filter.

## Important v19 semantics

- `health` describes the decision/state of an update: `ok`, `degraded`, `fault`, `disabled`, or `rejected`.
- `check_flags` describes which health/plausibility checks were triggered. It is intentionally not named `fault_flags`, because the same flags can explain degradation, rejection, or true sensor/data faults.
- `shaft_kf_event` is an optional per-update debug message. It can be disabled with `SKF_DBG_EVENT=0`.
- `shaft_kf_status` contains cumulative statistics from filter start and is published together with `shaft_kf_estimate`.
- VL-X and VL-Y are treated as body-frame FRD measurements. There is no axis remapping. Sign convention is configured using `SKF_VL_SGN_X` and `SKF_VL_SGN_Y`.
- The measurement model for VL projects local NED state to body frame using yaw:
  - VL-X: `x_body = cos(yaw)*x_N + sin(yaw)*y_E`
  - VL-Y: `y_body = -sin(yaw)*x_N + cos(yaw)*y_E`

## Callback design

`Run()` is prepared for multiple `registerCallback()` sources. Register callbacks for VL-X, VL-Y, PAT and optionally barometer. In each `Run()` execution, update all available subscriptions and call the corresponding `fuse*()` methods. If several inputs arrived before one `Run()`, all of them can be processed before publishing estimate/status.

## TODO before building in your PX4 tree

1. Copy `msg/*.msg` into the PX4 `msg/` directory or project-specific message location.
2. Register new uORB messages in PX4 build files.
3. Replace TODO subscription declarations with your actual custom topic names.
4. Add the module directory to the PX4 module build list.
5. Expand health checks to fully match the MATLAB v17/v15 logic.
