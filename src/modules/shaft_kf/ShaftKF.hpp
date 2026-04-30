#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <stdint.h>

class ShaftKF
{
public:
    static constexpr int STATE_SIZE = 7;
    static constexpr int STATE_DIM  = STATE_SIZE;

    enum StateIndex : uint8_t {
        PX   = 0,
        PY   = 1,
        PZ   = 2,
        VX   = 3,
        VY   = 4,
        VZ   = 5,
        DYAW = 6
    };

    enum class EventType : uint8_t {
        None        = 0,
        VL_X        = 1,
        VL_Y        = 2,
        PAT_Z       = 3,
        PAT_DYAW    = 4,
        BARO_Z      = 5,
        XY_ZUPT_X   = 6,
        XY_ZUPT_Y   = 7,
        Z_ZUPT      = 8,
    };

    enum class Health : uint8_t {
        Unknown  = 0,
        Ok       = 1,
        Degraded = 2,
        Rejected = 3,
        Fault    = 4,
        Disabled = 5
    };

    enum CheckFlags : uint16_t {
        CHECK_NONE              = 0,
        CHECK_NIS               = 1u << 0,
        CHECK_RANGE_STATUS      = 1u << 1,
        CHECK_HIGH_SIGMA        = 1u << 2,
        CHECK_LOW_SIGNAL        = 1u << 3,
        CHECK_HIGH_AMBIENT      = 1u << 4,
        CHECK_RAW_VELOCITY      = 1u << 5,
        CHECK_RAW_ACCELERATION  = 1u << 6,
        CHECK_RADIAL_LIMIT      = 1u << 7,
        CHECK_SQUAL2            = 1u << 8,
        CHECK_FREEZE            = 1u << 9,
        CHECK_SETPOINT_MISMATCH = 1u << 10,
        CHECK_TIMEOUT           = 1u << 11,
        CHECK_SHUTTER           = 1u << 12,
    };

    struct Params
    {
        // --- initial covariance ---
        float p0_px{0.01f * 0.01f};
        float p0_py{0.01f * 0.01f};
        float p0_pz{0.02f * 0.02f};

        float p0_vx{0.05f * 0.05f};
        float p0_vy{0.05f * 0.05f};
        float p0_vz{0.02f * 0.02f};

        float p0_dyaw{math::radians(10.0f) * math::radians(10.0f)};

        // --- process noise ---
        float sigma_ax{0.6f};
        float sigma_ay{0.6f};
        float sigma_az{1.5f};
        float sigma_dyaw{math::radians(0.5f)};
        // float max_dt{0.20f};

        // --- VL ---
        bool  vl_enabled{true};
        uint8_t vl_offset_x_mm{0};  // center offset for physical VL-X [mm]
        uint8_t vl_offset_y_mm{0};  // center offset for physical VL-Y [mm]

        // MATLAB v17 convention
        float vl_sign_body_x{1.0f}; // centered distance -> body X displacement sign
        float vl_sign_body_y{1.0f}; // centered distance -> body Y displacement sign

        bool  vl_use_dyaw_in_rotation{false};   // include dYaw in VL yaw projection only after validation
        float vl_trolley_z_offset{0.05f};

        float vl_sigma_floor{0.005f};
        float vl_sigma_roll{math::radians(1.0f)};
        float vl_sigma_pitch{math::radians(1.0f)};
        float vl_time_skew_sigma{0.010f};
        float vl_weak_signal_mult{3.0f};
        float vl_nis_gate{9.0f};

        bool  vl_health_enabled{true};
        bool  vl_reject_on_fault{true};
        float vl_degraded_R_mult{5.0f};
        float vl_fault_R_mult{50.0f};

        bool    vl_use_range_status{true};
        int32_t vl_good_range_status{0};

        bool  vl_use_sigma{true};
        float vl_sigma_degraded_mm{10.0f};
        float vl_sigma_fault_mm{30.0f};

        bool  vl_use_signal_spad{true};
        float vl_signal_degraded{3000.0f};
        float vl_signal_fault{1000.0f};

        bool  vl_use_ambient_spad{true};
        float vl_ambient_degraded{5.0f};
        float vl_ambient_fault{10.0f};

        bool  vl_use_raw_velocity{true};
        float vl_raw_v_degraded{0.4f};
        float vl_raw_v_fault{1.0f};

        bool  vl_use_raw_accel{false};
        float vl_raw_a_degraded{3.0f};
        float vl_raw_a_fault{8.0f};

        bool  vl_use_radial_limit{true};
        float vl_radial_degraded{0.08f};
        float vl_radial_fault{0.15f};

        bool  vl_use_setpoint{true};
        float vl_vxy_sp_degraded_diff{0.5f};
        float vl_vxy_sp_fault_diff{1.0f};

        // --- XY ZUPT ---
        bool  xy_zupt_enabled{true};
        float xy_zupt_max_vxy_sp{0.05f};
        float xy_zupt_max_tilt{math::radians(3.0f)};
        float xy_zupt_min_interval{0.05f};
        float xy_zupt_sigma_vxy0{0.08f};
        float xy_zupt_nis_gate{9.0f};

        // --- PAT ---
        bool  pat_enabled{true};
        bool  pat_use_resolution_cpi{true};
        float pat_m_per_count_x{1.0e-4f};
        float pat_m_per_count_y{1.0e-4f};

        float pat_axis_angle{0.0f};
        float pat_sign_z{1.0f};

        float pat_sigma_z0{0.010f};
        float pat_squal_ref{40.0f};
        uint8_t pat_squal_min{5};

        float pat_scale_sigma_rel{0.02f};
        float pat_time_skew_sigma{0.005f};
        float pat_nis_gate_z{9.0f};

        bool  pat_enable_zero_vz{true};
        float pat_no_new_zupt_timeout{0.20f};
        float pat_comm_timeout{0.50f};

        float pat_sigma_vz0{0.03f};
        float pat_nis_gate_vz0{9.0f};

        bool  pat_health_enabled{true};
        bool  pat_reject_on_fault{true};
        float pat_degraded_R_mult{10.0f};
        float pat_fault_R_mult{100.0f};

        bool pat_no_new_data_degraded{false};

        bool  pat_use_squal2{true};
        uint8_t pat_squal2_degraded{40};
        uint8_t pat_squal2_fault{25};

        bool  pat_use_shutter{false};
        uint16_t pat_shutter_degraded{220};
        uint16_t pat_shutter_fault{260};

        bool  pat_use_raw_vz{true};
        float pat_raw_vz_degraded{1.0f};
        float pat_raw_vz_fault{1.8f};

        bool  pat_use_setpoint{true};
        float pat_freeze_expected_vz{0.35f};
        float pat_freeze_raw_vz{0.05f};
        float pat_vz_sp_degraded_diff{0.7f};
        float pat_vz_sp_fault_diff{1.2f};

        // --- gating ---
        bool    use_takeoff_gate{true};
        int32_t takeoff_state_flight{5};
        float   enable_after_flight_delay{0.50f};
        bool    allow_checks_without_takeoff{true};

        // --- yaw ---
        bool  yaw_enabled{true};
        float yaw_sign{1.0f};
        float yaw_initial{0.0f};
        float yaw_rope_radius{0.003f};

        float yaw_sigma_floor{math::radians(1.0f)};
        uint8_t yaw_squal_ref{40};
        uint8_t yaw_squal_min{5};
        float yaw_nis_gate{9.0f};

        // --- baro ---
        bool  baro_enabled{true};
        bool  baro_z_positive_down{true};
        bool  baro_use_first_as_ref{true};

        float baro_alt0{NAN};
        float baro_sigma_z{0.50f};
        float baro_nis_gate{16.0f};

        float min_R{1.0e-9f};
    };

    // --- samples ---
    struct AttitudeSample {
        float roll{0.f};
        float pitch{0.f};
        float yaw{0.f};
    };

    struct SetpointSample {
        bool  valid{false};
        float vx{NAN};
        float vy{NAN};
        float vz{NAN};
    };

    struct TakeoffSample {
        bool         valid{false};
        uint8_t      state{0};
        hrt_abstime  first_flight_time{0};
    };

    struct VLSample {
        hrt_abstime timestamp_sample{0};
        uint16_t distance_mm{0};
        uint16_t sigma_mm{0};
        uint32_t signal_per_spad_kcps{0};
        uint32_t ambient_per_spad_kcps{0};
        uint16_t number_of_spad{0};
        uint8_t range_status{0};
    };

    struct PATSample {
        hrt_abstime timestamp_sample{0};
        int64_t x_sum{0};
        int64_t y_sum{0};
        uint16_t x_resolution_cpi{0};
        uint16_t y_resolution_cpi{0};
        uint16_t shutter{0};
        uint8_t squal{0};
        uint8_t squal2{0};
        bool  new_data{false};
    };

    struct BaroSample {
        hrt_abstime timestamp_sample{0};
        float baro_alt_meter{NAN};
    };

    struct UpdateInfo {
		EventType event{EventType::None};
		Health health{Health::Unknown};
		hrt_abstime timestamp_sample{0};
        uint32_t sequence{0};
		bool accepted{false};
		float measurement{NAN};
		float prediction{NAN};
		float innovation{NAN};
		float innovation_var{NAN};
		float nis{NAN};
		float r{NAN};
		float h0{0.f};
		float h1{0.f};
		matrix::Vector<float, STATE_DIM> k{};
		matrix::Vector<float, STATE_DIM> state_after{};
		matrix::Vector<float, STATE_DIM> p_diag_after{};
        matrix::Vector<float, 3> p_v_diag_after{};
		uint16_t check_flags{0};
	};

    struct StatusCounters {
		uint32_t prediction_count{0};
		uint32_t event_count{0};
		uint32_t update_count{0};
		uint32_t accepted_update_count{0};
		uint32_t rejected_update_count{0};
		uint32_t vl_x_update_count{0};
		uint32_t vl_y_update_count{0};
		uint32_t pat_z_update_count{0};
		uint32_t pat_dyaw_update_count{0};
		uint32_t baro_z_update_count{0};
		uint32_t xy_zupt_update_count{0};
		uint32_t z_zupt_update_count{0};
		uint32_t vl_x_reject_count{0};
		uint32_t vl_y_reject_count{0};
		uint32_t pat_z_reject_count{0};
		uint32_t pat_dyaw_reject_count{0};
		uint32_t baro_z_reject_count{0};
		uint32_t xy_zupt_reject_count{0};
		uint32_t z_zupt_reject_count{0};
		uint32_t vl_x_degraded_count{0};
		uint32_t vl_y_degraded_count{0};
		uint32_t pat_z_degraded_count{0};
		uint32_t pat_dyaw_degraded_count{0};
		uint32_t baro_z_degraded_count{0};
		uint16_t event_mask{0};
		uint16_t accepted_event_mask{0};
		uint16_t rejected_event_mask{0};
		uint16_t degraded_event_mask{0};
		uint16_t fault_event_mask{0};
		uint16_t vl_x_fault_flags{0};
		uint16_t vl_y_fault_flags{0};
		uint16_t pat_fault_flags{0};
		uint16_t baro_fault_flags{0};
		float max_nis{0.f};
		float mean_nis{0.f};
		float max_abs_innovation{0.f};
		float last_vl_x_nis{NAN};
		float last_vl_y_nis{NAN};
		float last_pat_z_nis{NAN};
		float last_pat_dyaw_nis{NAN};
		float last_baro_z_nis{NAN};
		float last_vl_x_innovation{NAN};
		float last_vl_y_innovation{NAN};
		float last_pat_z_innovation{NAN};
		float last_pat_dyaw_innovation{NAN};
		float last_baro_z_innovation{NAN};
		Health vl_x_health{Health::Unknown};
		Health vl_y_health{Health::Unknown};
		Health pat_health{Health::Unknown};
		Health baro_health{Health::Unknown};
	};

    // API
    bool initialized() const { return _initialized; }
    void reset(const Params &params, hrt_abstime now);
    void setParams(const Params &params) { _params = params; }
    const Params &params() const { return _params; }


    void predictTo(hrt_abstime timestamp_sample);



    UpdateInfo fuseVlX(const VLSample &s, const AttitudeSample &att, const SetpointSample &sp, const TakeoffSample &tko);
    UpdateInfo fuseVlY(const VLSample &s, const AttitudeSample &att, const SetpointSample &sp, const TakeoffSample &tko);
    UpdateInfo fusePAT(const PATSample &s, const AttitudeSample &att, const SetpointSample &sp, const TakeoffSample &tko);
    UpdateInfo fuseBaro(const BaroSample &s);
    UpdateInfo maybeFuseXyZupt(hrt_abstime now, const AttitudeSample &att, const SetpointSample &sp, const TakeoffSample &tko);

    const matrix::Vector<float, STATE_DIM> &state() const { return _x; }
	const matrix::SquareMatrix<float, STATE_DIM> &covariance() const { return _P; }
	hrt_abstime lastPredictTime() const { return _last_predict_us; }
    const StatusCounters &status() const { return _status; }

private:
    UpdateInfo fuseVLBody(const VLSample &s, EventType event, bool bx, const AttitudeSample &att, const SetpointSample &sp, const TakeoffSample &tko);
    void predict(float dt);
    bool updateScalar(float z, const matrix::Vector<float, STATE_DIM> &H, float R, float nis_gate, UpdateInfo &info);
	void symmetrizeP();
	bool setpointChecksActive(hrt_abstime now, const TakeoffSample &tko) const;
	float healthRFactor(Health h, float degraded_mult, float fault_mult) const;
	Health combineHealth(bool fault, bool degraded) const;
    static float sq(float v) { return v * v; }
	static float safeFinite(float v, float fallback) { return PX4_ISFINITE(v) ? v : fallback; }

    void finishInfo(UpdateInfo &info, float z, const matrix::Vector<float, STATE_DIM> &H, float R);
    void accountEvent(const UpdateInfo &i);


    struct AxisHistory {
        bool         valid{false};
        float        last_z{NAN};
        float        last_v{NAN};
        hrt_abstime  last_t{0};
    };

    Params _params{};
	matrix::Vector<float, STATE_SIZE> _x{};
	matrix::SquareMatrix<float, STATE_SIZE> _P{};
    hrt_abstime _last_predict_us{0};
    StatusCounters _status{};

    bool _initialized{false};
    uint32_t _seq{0};

    AxisHistory _vl_x_hist{}; // physical/body VL-X history
	AxisHistory _vl_y_hist{}; // physical/body VL-Y history
	hrt_abstime _last_xy_zupt{0};

	float _last_pat_vertical{NAN};
	float _last_pat_tangential{NAN};
	hrt_abstime _last_pat_time{0};
	hrt_abstime _last_pat_new_data_time{0};
	hrt_abstime _last_pat_comm_time{0};
	Health _pat_health{Health::Unknown};
	uint32_t _pat_fault_flags{0};
	float _pat_raw_vz{NAN};
	float _pat_raw_yaw_rate{NAN};

	bool _baro_ref_valid{false};
	float _baro_alt0{NAN};
};
