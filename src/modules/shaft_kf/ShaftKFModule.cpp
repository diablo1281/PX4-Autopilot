#include "ShaftKF.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/shaft_kf_estimate.h>
#include <uORB/topics/shaft_kf_status.h>
#include <uORB/topics/shaft_kf_event.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/distance_sensor_single.h>
#include <uORB/topics/optical_navigation_vertical.h>
#include <uORB/topics/vehicle_odometry.h>

using matrix::Eulerf;
using matrix::Quatf;
using namespace time_literals;

class ShaftKFModule : public ModuleBase<ShaftKFModule>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ShaftKFModule();
	~ShaftKFModule() override;

	static int task_spawn(int argc, char *argv[]);
	static ShaftKFModule *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();


private:
	void Run() override;

	void parameters_update();

	void registerCallbacks();
	void unregisterCallbacks();

	void publishEstimate(uint64_t now);
	void publishStatus(uint64_t now);
	void publishEvent(const ShaftKF::UpdateInfo &info);

	ShaftKF::AttitudeSample readAttitude();
	ShaftKF::SetpointSample readSetpoint();
	ShaftKF::TakeoffSample readTakeoffStatus();
	bool readVlX(ShaftKF::VLSample &sample);
	bool readVlY(ShaftKF::VLSample &sample);
	bool readPat(ShaftKF::PATSample &sample);
	bool readBaro(ShaftKF::BaroSample &sample);

	enum class UpdateEvent : uint8_t {
		VL_X	= 0,
		VL_Y	= 1,
		PAT		= 2,
		BARO	= 3,
		Count,
		None	= 255
	};

	ShaftKF _filter{};
	ShaftKF::Params _cfg{};
	bool _reset_requested{true};
	bool _callbacks_registered{false};

	uORB::Publication<shaft_kf_estimate_s> _estimate_pub{ORB_ID(shaft_kf_estimate)};
	uORB::Publication<shaft_kf_status_s> _status_pub{ORB_ID(shaft_kf_status)};
	uORB::Publication<shaft_kf_event_s> _event_pub{ORB_ID(shaft_kf_event)};
	uORB::Publication<vehicle_odometry_s> _vehicle_odometry_pub{ORB_ID(vehicle_visual_odometry)};

	uORB::SubscriptionCallbackWorkItem _vl_x_sub{this, ORB_ID(distance_sensor_single), 0};
	uORB::SubscriptionCallbackWorkItem _vl_y_sub{this, ORB_ID(distance_sensor_single), 1};
	uORB::SubscriptionCallbackWorkItem _pat_sub{this, ORB_ID(optical_navigation_vertical)};
	uORB::SubscriptionCallbackWorkItem _baro_sub{this, ORB_ID(vehicle_air_data)};

	// Non-callback auxiliary sources.
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _takeoff_status_sub{ORB_ID(takeoff_status)};

	ShaftKF::AttitudeSample _last_att{};
	ShaftKF::SetpointSample _last_sp{};
	ShaftKF::TakeoffSample _last_tko{};

	ShaftKF::VLSample vlx_sample{}, vly_sample{};
	ShaftKF::PATSample pat_sample{};
	ShaftKF::BaroSample baro_sample{};
	ShaftKF::UpdateInfo info{};

	hrt_abstime events_timestamps[static_cast<uint8_t>(UpdateEvent::Count)] = {};
	UpdateEvent current_event = UpdateEvent::None;

	hrt_abstime _last_publish_us{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SKF_P0_XY>) _param_p0_xy,
		(ParamFloat<px4::params::SKF_P0_Z>) _param_p0_z,
		(ParamFloat<px4::params::SKF_P0_VXY>) _param_p0_vxy,
		(ParamFloat<px4::params::SKF_P0_VZ>) _param_p0_vz,
		(ParamFloat<px4::params::SKF_P0_DYAW>) _param_p0_dyaw,
		(ParamFloat<px4::params::SKF_SIGMA_AX>) _param_sigma_ax,
		(ParamFloat<px4::params::SKF_SIGMA_AY>) _param_sigma_ay,
		(ParamFloat<px4::params::SKF_SIGMA_AZ>) _param_sigma_az,
		(ParamFloat<px4::params::SKF_SIGMA_DYAW>) _param_sigma_dyaw,
		(ParamFloat<px4::params::SKF_MAX_DT>) _param_max_dt,
		(ParamInt<px4::params::SKF_VL_OFF_X>) _param_vl_off_x,
		(ParamInt<px4::params::SKF_VL_OFF_Y>) _param_vl_off_y,
		(ParamFloat<px4::params::SKF_VL_SGN_X>) _param_vl_sgn_x,
		(ParamFloat<px4::params::SKF_VL_SGN_Y>) _param_vl_sgn_y,
		(ParamBool<px4::params::SKF_VL_DYAW_ROT>) _param_vl_dyaw_rot,
		(ParamFloat<px4::params::SKF_VL_LEVER>) _param_vl_lever,
		(ParamFloat<px4::params::SKF_VL_SIGMA_FL>) _param_vl_sig_floor,
		(ParamFloat<px4::params::SKF_VL_NIS>) _param_vl_nis,
		(ParamBool<px4::params::SKF_VL_HEALTH_EN>) _param_vl_health_en,
		(ParamBool<px4::params::SKF_VL_REJECT_EN>) _param_vl_reject_en,
		(ParamFloat<px4::params::SKF_VL_SIGMA_DGR>) _param_vl_sigma_degraded,
		(ParamFloat<px4::params::SKF_VL_SIGMA_FLT>) _param_vl_sigma_fault,
		(ParamFloat<px4::params::SKF_VL_V_DGR>) _param_vl_v_degraded,
		(ParamFloat<px4::params::SKF_VL_V_FLT>) _param_vl_v_fault,
		(ParamBool<px4::params::SKF_VL_RAD_EN>) _param_vl_radial_limit_en,
		(ParamFloat<px4::params::SKF_VL_RAD_DGR>) _param_vl_radial_degraded,
		(ParamFloat<px4::params::SKF_VL_RAD_FLT>) _param_vl_radial_fault,
		(ParamBool<px4::params::SKF_XY_ZUPT_EN>) _param_xy_zupt_en,
		(ParamFloat<px4::params::SKF_XY_ZUPT_SIG>) _param_xy_zupt_sig,
		(ParamFloat<px4::params::SKF_PAT_ROT_DEG>) _param_pat_rot_deg,
		(ParamFloat<px4::params::SKF_PAT_SIGMA_Z>) _param_pat_sigma_z,
		(ParamFloat<px4::params::SKF_PAT_NIS_Z>) _param_pat_nis_z,
		(ParamBool<px4::params::SKF_PAT_HLTH_EN>) _param_pat_health_en,
		(ParamBool<px4::params::SKF_PAT_REJ_EN>) _param_pat_reject_en,
		(ParamFloat<px4::params::SKF_PAT_SQ2_DGR>) _param_pat_sq2_degraded,
		(ParamFloat<px4::params::SKF_PAT_SQ2_FLT>) _param_pat_sq2_fault,
		(ParamFloat<px4::params::SKF_PAT_VZ_DGR>) _param_pat_vz_degraded,
		(ParamFloat<px4::params::SKF_PAT_VZ_FLT>) _param_pat_vz_fault,
		(ParamFloat<px4::params::SKF_YAW_RAD>) _param_yaw_rad,
		(ParamFloat<px4::params::SKF_YAW_SIGMA>) _param_yaw_sigma,
		(ParamFloat<px4::params::SKF_BARO_SIGMA>) _param_baro_sigma,
		(ParamBool<px4::params::SKF_BARO_EN>) _param_baro_en,
		(ParamBool<px4::params::SKF_DBG_EVENT>) _param_dbg_event
	)
};

ShaftKFModule::ShaftKFModule() :
	ModuleParams(nullptr),
	// ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::INS3)
{
	parameters_update();

	_estimate_pub.advertise();
	_status_pub.advertise();
	_event_pub.advertise();
	_vehicle_odometry_pub.advertise();
}

ShaftKFModule::~ShaftKFModule()
{
	unregisterCallbacks();
}

void ShaftKFModule::parameters_update()
{
	updateParams();

	_cfg.p0_px = _cfg.p0_py = _param_p0_xy.get();
	_cfg.p0_pz = _param_p0_z.get();
	_cfg.p0_vx = _cfg.p0_vy = _param_p0_vxy.get();
	_cfg.p0_vz = _param_p0_vz.get();
	_cfg.p0_dyaw = _param_p0_dyaw.get();

	_cfg.sigma_ax = _param_sigma_ax.get();
	_cfg.sigma_ay = _param_sigma_ay.get();
	_cfg.sigma_az = _param_sigma_az.get();
	_cfg.sigma_dyaw = _param_sigma_dyaw.get();
	// _cfg.max_dt = _param_max_dt.get();

	_cfg.vl_offset_x_mm = _param_vl_off_x.get();
	_cfg.vl_offset_y_mm = _param_vl_off_y.get();
	_cfg.vl_sign_body_x = _param_vl_sgn_x.get();
	_cfg.vl_sign_body_y = _param_vl_sgn_y.get();
	_cfg.vl_use_dyaw_in_rotation = _param_vl_dyaw_rot.get();
	_cfg.vl_trolley_z_offset = _param_vl_lever.get();
	_cfg.vl_sigma_floor = _param_vl_sig_floor.get();
	_cfg.vl_nis_gate = _param_vl_nis.get();

	_cfg.vl_health_enabled = _param_vl_health_en.get();
	_cfg.vl_reject_on_fault = _param_vl_reject_en.get();
	_cfg.vl_sigma_degraded_mm = _param_vl_sigma_degraded.get();
	_cfg.vl_sigma_fault_mm = _param_vl_sigma_fault.get();
	_cfg.vl_raw_v_degraded = _param_vl_v_degraded.get();
	_cfg.vl_raw_v_fault = _param_vl_v_fault.get();
	_cfg.vl_use_radial_limit = _param_vl_radial_limit_en.get();
	_cfg.vl_radial_degraded = _param_vl_radial_degraded.get();
	_cfg.vl_radial_fault = _param_vl_radial_fault.get();

	_cfg.xy_zupt_enabled = _param_xy_zupt_en.get();
	_cfg.xy_zupt_sigma_vxy0 = _param_xy_zupt_sig.get();

	_cfg.pat_axis_angle = math::radians(_param_pat_rot_deg.get());
	_cfg.pat_sigma_z0 = _param_pat_sigma_z.get();
	_cfg.pat_nis_gate_z = _param_pat_nis_z.get();
	_cfg.pat_health_enabled = _param_pat_health_en.get();
	_cfg.pat_reject_on_fault = _param_pat_reject_en.get();
	_cfg.pat_squal2_degraded = _param_pat_sq2_degraded.get();
	_cfg.pat_squal2_fault = _param_pat_sq2_fault.get();
	_cfg.pat_raw_vz_degraded = _param_pat_vz_degraded.get();
	_cfg.pat_raw_vz_fault = _param_pat_vz_fault.get();

	_cfg.yaw_rope_radius = _param_yaw_rad.get();
	_cfg.yaw_sigma_floor = _param_yaw_sigma.get();

	_cfg.baro_sigma_z = _param_baro_sigma.get();
	_cfg.baro_enabled = _param_baro_en.get();

	_filter.setParams(_cfg);
}

ShaftKF::AttitudeSample ShaftKFModule::readAttitude()
{
	vehicle_attitude_s msg{};
	if (_vehicle_attitude_sub.update(&msg)) {
		const Quatf q(msg.q);
		const Eulerf e(q);
		_last_att.roll = e.phi(); _last_att.pitch = e.theta(); _last_att.yaw = e.psi();
	}
	return _last_att;
}

ShaftKF::SetpointSample ShaftKFModule::readSetpoint()
{
	vehicle_local_position_setpoint_s msg{};
	if (_vehicle_local_position_setpoint_sub.update(&msg)) {
		_last_sp.valid = true; _last_sp.vx = msg.vx; _last_sp.vy = msg.vy; _last_sp.vz = msg.vz;
	}
	return _last_sp;
}

ShaftKF::TakeoffSample ShaftKFModule::readTakeoffStatus()
{
	takeoff_status_s msg{};
	if (_takeoff_status_sub.update(&msg)) {
		_last_tko.valid = true;
		_last_tko.state = msg.takeoff_state;
		if (msg.takeoff_state == static_cast<uint8_t>(_cfg.takeoff_state_flight) && _last_tko.first_flight_time == 0) {
			_last_tko.first_flight_time = msg.timestamp;
		}
	}
	return _last_tko;
}

bool ShaftKFModule::readVlX(ShaftKF::VLSample &s)
{
	distance_sensor_single_s msg{};
	if (_vl_x_sub.update(&msg)) {
		s.timestamp_sample = msg.timestamp_sample != 0 ? msg.timestamp_sample : msg.timestamp;
		s.distance_mm = msg.distance_mm; s.sigma_mm = msg.sigma_mm;
		s.signal_per_spad_kcps = msg.signal_per_spad_kcps; s.ambient_per_spad_kcps = msg.ambient_per_spad_kcps;
		s.number_of_spad = msg.number_of_spad; s.range_status = msg.range_status;
		return true;
	}
	return false;
}

bool ShaftKFModule::readVlY(ShaftKF::VLSample &s)
{
	distance_sensor_single_s msg{};
	if (_vl_y_sub.update(&msg)) {
		s.timestamp_sample = msg.timestamp_sample != 0 ? msg.timestamp_sample : msg.timestamp;
		s.distance_mm = msg.distance_mm; s.sigma_mm = msg.sigma_mm;
		s.signal_per_spad_kcps = msg.signal_per_spad_kcps; s.ambient_per_spad_kcps = msg.ambient_per_spad_kcps;
		s.number_of_spad = msg.number_of_spad; s.range_status = msg.range_status;
		return true;
	}
	return false;
}

bool ShaftKFModule::readPat(ShaftKF::PATSample &s)
{
	optical_navigation_vertical_s msg{};
	if (_pat_sub.update(&msg)) {
		s.timestamp_sample = msg.timestamp_sample != 0 ? msg.timestamp_sample : msg.timestamp;
		s.x_sum = msg.x_sum; s.y_sum = msg.y_sum;
		s.x_resolution_cpi = msg.x_resolution; s.y_resolution_cpi = msg.y_resolution;
		s.shutter = msg.shutter; s.squal = msg.squal; s.squal2 = msg.squal2; s.new_data = msg.new_data;
		return true;
	}
	return false;
}

bool ShaftKFModule::readBaro(ShaftKF::BaroSample &s)
{
	vehicle_air_data_s msg{};
	if (_baro_sub.update(&msg)) {
		s.timestamp_sample = msg.timestamp_sample != 0 ? msg.timestamp_sample : msg.timestamp;
		s.baro_alt_meter = msg.baro_alt_meter;
		return true;
	}
	return false;
}

bool ShaftKFModule::init()
{
	parameters_update();
	_filter.reset(_cfg, hrt_absolute_time());
	registerCallbacks();
	// ScheduleOnInterval(20_ms); // safety fallback if no callback source is registered yet
	ScheduleDelayed(1000_ms);
	return true;
}

void ShaftKFModule::registerCallbacks()
{
	const bool vlx_ok = _vl_x_sub.registerCallback();
	const bool vly_ok = _vl_y_sub.registerCallback();
	const bool pat_ok = _pat_sub.registerCallback();
	const bool baro_ok = _baro_sub.registerCallback();

	_callbacks_registered = vlx_ok && vly_ok && pat_ok && baro_ok;

	if (!_callbacks_registered) {
		PX4_ERR("Callback registration failed: vlx=%d vly=%d pat=%d baro=%d",
            vlx_ok, vly_ok, pat_ok, baro_ok);
		return;
	}

	return;
}

void ShaftKFModule::unregisterCallbacks()
{
	_vl_x_sub.unregisterCallback();
	_vl_y_sub.unregisterCallback();
	_pat_sub.unregisterCallback();
	_baro_sub.unregisterCallback();
	_callbacks_registered = false;
}

void ShaftKFModule::Run()
{
	if (should_exit()) {
		unregisterCallbacks();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated() || !_callbacks_registered) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		parameters_update();
	}

	if (!_callbacks_registered) {

		registerCallbacks();

		if (!_callbacks_registered) {
			ScheduleDelayed(1000_ms);
			return;
		}
	}

	const uint64_t now = hrt_absolute_time();
	bool pub_event = _param_dbg_event.get();

	readAttitude();
	readSetpoint();
	readTakeoffStatus();

	uint8_t event_mask = 0;
	// hrt_abstime events_timestamps[static_cast<uint8_t>(UpdateEvent::Count)] = {};
	// UpdateEvent current_event = UpdateEvent::None;
	// ShaftKF::VLSample vlx_sample, vly_sample;
	// ShaftKF::PATSample pat_sample;
	// ShaftKF::BaroSample baro_sample;
	// ShaftKF::UpdateInfo info{};

	if (readVlX(vlx_sample)) {
		event_mask |= (1 << static_cast<uint8_t>(UpdateEvent::VL_X));
		events_timestamps[static_cast<uint8_t>(UpdateEvent::VL_X)] = vlx_sample.timestamp_sample;
	}
	if (readVlY(vly_sample)) {
		event_mask |= (1 << static_cast<uint8_t>(UpdateEvent::VL_Y));
		events_timestamps[static_cast<uint8_t>(UpdateEvent::VL_Y)] = vly_sample.timestamp_sample;
	}
	if (readPat(pat_sample)) {
		event_mask |= (1 << static_cast<uint8_t>(UpdateEvent::PAT));
		events_timestamps[static_cast<uint8_t>(UpdateEvent::PAT)] = pat_sample.timestamp_sample;
	}
	if (readBaro(baro_sample)) {
		event_mask |= (1 << static_cast<uint8_t>(UpdateEvent::BARO));
		events_timestamps[static_cast<uint8_t>(UpdateEvent::BARO)] = baro_sample.timestamp_sample;
	}

	if(hrt_elapsed_time(&_last_publish_us) > 1000_ms) {
		_last_publish_us = now;
		PX4_INFO("mask=%u vlx=%llu vly=%llu pat=%llu baro=%llu",
			event_mask,
			events_timestamps[0],
			events_timestamps[1],
			events_timestamps[2],
			events_timestamps[3]);
	}

	while(event_mask > 0) {
		// Find event with oldest timestamp
		hrt_abstime oldest_timestamp = UINT64_MAX;
		for(uint8_t i = 0; i < static_cast<uint8_t>(UpdateEvent::Count); i++) {
			if ((event_mask & (1 << i)) && events_timestamps[i] < oldest_timestamp) {
				oldest_timestamp = events_timestamps[i];
				current_event = static_cast<UpdateEvent>(i);
			}
		}

		// Process the event
		switch (current_event) {
		case UpdateEvent::VL_X:
			info = _filter.fuseVlX(vlx_sample, _last_att, _last_sp, _last_tko);
			if (pub_event) { publishEvent(info); }
			break;
		case UpdateEvent::VL_Y:
			info = _filter.fuseVlY(vly_sample, _last_att, _last_sp, _last_tko);
			if (pub_event) { publishEvent(info); }
			break;
		case UpdateEvent::PAT:
			info = _filter.fusePAT(pat_sample, _last_att, _last_sp, _last_tko);
			if (pub_event) { publishEvent(info); }
			break;
		case UpdateEvent::BARO:
			info = _filter.fuseBaro(baro_sample);
			if (pub_event) { publishEvent(info); }
			break;
		default:
			break;
		}

		// Clear the processed event
		event_mask &= ~(1 << static_cast<uint8_t>(current_event));
		events_timestamps[static_cast<uint8_t>(current_event)] = UINT64_MAX; // mark as processed
	}

	info = _filter.maybeFuseXyZupt(now, _last_att, _last_sp, _last_tko);
	if (pub_event) { publishEvent(info); }

	publishEstimate(now);
	publishStatus(now);

	memset(events_timestamps, 0, sizeof(events_timestamps));

	// ScheduleDelayed(100_ms);
}

void ShaftKFModule::publishEvent(const ShaftKF::UpdateInfo &e)
{
	shaft_kf_event_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.timestamp_sample = e.timestamp_sample;
	msg.sequence = e.sequence;

	msg.measurement = e.measurement;
	msg.prediction = e.prediction;
	msg.innovation = e.innovation;
	msg.innovation_var = e.innovation_var;
	msg.nis = e.nis;
	msg.r = e.r;

	msg.h0 = e.h0;
	msg.h1 = e.h1;

	for (int i = 0; i < ShaftKF::STATE_SIZE; ++i) {
		msg.k[i] = e.k(i);
		msg.state_after[i] = e.state_after(i);
		msg.p_diag_after[i] = e.p_diag_after(i);
		if(i < 3) {
			msg.p_v_diag_after[i] = e.p_v_diag_after(i);
		}
	}

	msg.event_type = static_cast<uint8_t>(e.event);
	msg.health = static_cast<uint8_t>(e.health);
	msg.check_flags = e.check_flags;

	_event_pub.publish(msg);
}

void ShaftKFModule::publishEstimate(uint64_t now)
{
	shaft_kf_estimate_s msg{};
	msg.timestamp = now;
	msg.timestamp_sample = _filter.lastPredictTime();
	const auto &x = _filter.state();
	const auto &P = _filter.covariance();
	msg.x = x(0); msg.y = x(1); msg.z = x(2);
	msg.vx = x(3); msg.vy = x(4); msg.vz = x(5);
	msg.dyaw = x(6);
	msg.var_x = P(0,0); msg.var_y = P(1,1); msg.var_z = P(2,2);
	msg.var_vx = P(3,3); msg.var_vy = P(4,4); msg.var_vz = P(5,5);
	msg.var_dyaw = P(6,6);
	msg.yaw_ekf = _last_att.yaw;
	_estimate_pub.publish(msg);

	vehicle_odometry_s vo_msg = {};
    vo_msg.timestamp = hrt_absolute_time();
    vo_msg.timestamp_sample = _filter.lastPredictTime();
    vo_msg.timestamp_sample_z = 0;

    vo_msg.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
    vo_msg.position[0] = x(0);
    vo_msg.position[1] = x(1);
    vo_msg.position[2] = x(2);

    vo_msg.q[0] = NAN;

    vo_msg.velocity_frame = vehicle_odometry_s::POSE_FRAME_NED;
    vo_msg.velocity[0] = x(3);
    vo_msg.velocity[1] = x(4);
    vo_msg.velocity[2] = x(5);

    vo_msg.angular_velocity[0] = NAN;
    vo_msg.angular_velocity[1] = NAN;
    vo_msg.angular_velocity[2] = NAN;

    vo_msg.position_variance[0] = P(0,0);
    vo_msg.position_variance[1] = P(1,1);
    vo_msg.position_variance[2] = P(2,2);

    vo_msg.velocity_variance[0] = P(3,3);
    vo_msg.velocity_variance[1] = P(4,4);
    vo_msg.velocity_variance[2] = P(5,5);

    vo_msg.reset_counter = 0;
    vo_msg.quality = 95;

    _vehicle_odometry_pub.publish(vo_msg);
}



void ShaftKFModule::publishStatus(uint64_t now)
{
	shaft_kf_status_s msg{};
	msg.timestamp = now;
	msg.timestamp_sample = _filter.lastPredictTime();
	const auto &s = _filter.status();

	msg.prediction_count = s.prediction_count;
	msg.event_count = s.event_count;
	msg.update_count = s.update_count;
	msg.accepted_update_count = s.accepted_update_count;
	msg.rejected_update_count = s.rejected_update_count;

	msg.vl_x_update_count = s.vl_x_update_count;
	msg.vl_y_update_count = s.vl_y_update_count;
	msg.pat_z_update_count = s.pat_z_update_count;
	msg.pat_dyaw_update_count = s.pat_dyaw_update_count;
	msg.baro_z_update_count = s.baro_z_update_count;
	msg.xy_zupt_update_count = s.xy_zupt_update_count;
	msg.z_zupt_update_count = s.z_zupt_update_count;

	msg.vl_x_reject_count = s.vl_x_reject_count;
	msg.vl_y_reject_count = s.vl_y_reject_count;
	msg.pat_z_reject_count = s.pat_z_reject_count;
	msg.pat_dyaw_reject_count = s.pat_dyaw_reject_count;
	msg.baro_z_reject_count = s.baro_z_reject_count;
	msg.xy_zupt_reject_count = s.xy_zupt_reject_count;
	msg.z_zupt_reject_count = s.z_zupt_reject_count;

	msg.vl_x_degraded_count = s.vl_x_degraded_count;
	msg.vl_y_degraded_count = s.vl_y_degraded_count;
	msg.pat_z_degraded_count = s.pat_z_degraded_count;
	msg.pat_dyaw_degraded_count = s.pat_dyaw_degraded_count;
	msg.baro_z_degraded_count = s.baro_z_degraded_count;

	msg.event_mask = s.event_mask;
	msg.accepted_event_mask = s.accepted_event_mask;
	msg.rejected_event_mask = s.rejected_event_mask;
	msg.degraded_event_mask = s.degraded_event_mask;
	msg.fault_event_mask = s.fault_event_mask;

	// msg.last_vl_x_check_flags = s.last_vl_x_check_flags;
	// msg.last_vl_y_check_flags = s.last_vl_y_check_flags;
	// msg.last_pat_check_flags = s.last_pat_check_flags;
	// msg.last_baro_check_flags = s.last_baro_check_flags;

	msg.max_nis = s.max_nis;
	msg.mean_nis = s.mean_nis;
	msg.max_abs_innovation = s.max_abs_innovation;

	msg.last_vl_x_nis = s.last_vl_x_nis;
	msg.last_vl_y_nis = s.last_vl_y_nis;
	msg.last_pat_z_nis = s.last_pat_z_nis;
	msg.last_pat_dyaw_nis = s.last_pat_dyaw_nis;
	msg.last_baro_z_nis = s.last_baro_z_nis;

	msg.vl_x_health = static_cast<uint8_t>(s.vl_x_health);
	msg.vl_y_health = static_cast<uint8_t>(s.vl_y_health);
	msg.pat_health = static_cast<uint8_t>(s.pat_health);
	msg.baro_health = static_cast<uint8_t>(s.baro_health);

	msg.debug_event_enabled = _param_dbg_event.get();

	_status_pub.publish(msg);
}



ShaftKFModule *ShaftKFModule::instantiate(int argc, char *argv[])
{
	ShaftKFModule *inst = new ShaftKFModule();
	if (inst && !inst->init()) { delete inst; inst = nullptr; }
	return inst;
}

int ShaftKFModule::task_spawn(int argc, char *argv[])
{
	PX4_INFO("Launching ShaftKF...");
	ShaftKFModule *instance = instantiate(argc, argv);
	if (instance) { _object.store(instance); _task_id = task_id_is_work_queue; return PX4_OK; }
	return PX4_ERROR;
}

int ShaftKFModule::print_status()
{
	PX4_INFO("ShaftKF running, events: %lu", _filter.status().event_count);
	return 0;
}

int ShaftKFModule::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }
	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Shaft Kalman Filter module.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("shaft_kf", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	return 0;
}

extern "C" __EXPORT int shaft_kf_main(int argc, char *argv[])
{
	return ShaftKFModule::main(argc, argv);
}
