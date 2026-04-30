#include "ShaftKF.hpp"

using matrix::Vector; using matrix::SquareMatrix;

void ShaftKF::reset(const Params &params, hrt_abstime now)
{
	_params = params;
	_x.zero();
	_x(DYAW) = _params.yaw_initial;
	_P.zero();
	_P(PX,PX)=_params.p0_px; _P(PY,PY)=_params.p0_py; _P(PZ,PZ)=_params.p0_pz;
	_P(VX,VX)=_params.p0_vx; _P(VY,VY)=_params.p0_vy; _P(VZ,VZ)=_params.p0_vz;
	_P(DYAW,DYAW)=_params.p0_dyaw;
	_last_predict_us = now;
	_initialized = true;
	_seq = 0;
	_status = StatusCounters{};
	_vl_x_hist = AxisHistory{};
	_vl_y_hist = AxisHistory{}; _last_xy_zupt = 0;
	_last_pat_vertical = NAN;
	_last_pat_tangential = NAN;
	_last_pat_time=0;
	_last_pat_new_data_time=0;
	_pat_health=Health::Unknown;
	_baro_ref_valid=false;
	_baro_alt0=NAN;
}

void ShaftKF::predictTo(hrt_abstime timestamp_sample)
{
	if (!_initialized) {
		reset(_params, timestamp_sample);
		return;
	}

	if (timestamp_sample <= _last_predict_us) { return; }

	float dt = (timestamp_sample - _last_predict_us) * 1.0e-6f;
	// FIXME Czemu jest stopniowy update z max_dt?
	// const float max_dt = math::max(_params.max_dt, 1.0e-4f);

	// while (dt > max_dt) {
	// 	predict(max_dt);
	// 	dt -= max_dt;
	// 	_last_predict_us += static_cast<hrt_abstime>(max_dt * 1.0e6f);
	// }

	predict(dt);
	_last_predict_us = timestamp_sample;
}

void ShaftKF::predict(float dt)
{
	if (dt <= 0.f) return;

	SquareMatrix<float, STATE_DIM> F;
	F.setIdentity();
	F(PX,VX)=dt;
	F(PY,VY)=dt;
	F(PZ,VZ)=dt;

	_x = F * _x;
	_x(DYAW) = matrix::wrap_pi(_x(DYAW));
	_P = F * _P * F.transpose();

	const float dt2 = dt * dt;
	const float dt3 = dt2 * dt;
	const float dt4 = dt2 * dt2;

	const float ax2 = sq(_params.sigma_ax);
	const float ay2 = sq(_params.sigma_ay);
	const float az2 = sq(_params.sigma_az);

	_P(PX,PX) += ax2 * dt4 * 0.25f;
	_P(PX,VX) += ax2 * dt3 * 0.5f;
	_P(VX,PX) += ax2 * dt3 * 0.5f;
	_P(VX,VX) += ax2 * dt2;

	_P(PY,PY) += ay2 * dt4 * 0.25f;
	_P(PY,VY) += ay2 * dt3 * 0.5f;
	_P(VY,PY) += ay2 * dt3 * 0.5f;
	_P(VY,VY) += ay2 * dt2;

	_P(PZ,PZ) += az2 * dt4 * 0.25f;
	_P(PZ,VZ) += az2 * dt3 * 0.5f;
	_P(VZ,PZ) += az2 * dt3 * 0.5f;
	_P(VZ,VZ) += az2 * dt2;

	_P(DYAW,DYAW) += sq(_params.sigma_dyaw) * dt;
	symmetrizeP();

	_status.prediction_count++;
}

void ShaftKF::symmetrizeP()
{
	_P = 0.5f * (_P + _P.transpose());

	for (int i = 0; i < STATE_DIM; i++) {
		_P(i,i) = math::max(_P(i,i), 1.0e-12f);
	}
}

ShaftKF::Health ShaftKF::combineHealth(bool fault, bool degraded) const
{
	if (fault) { return Health::Fault; }
	if (degraded) { return Health::Degraded; }
	return Health::Ok;
}

float ShaftKF::healthRFactor(Health h, float degraded_mult, float fault_mult) const
{
	switch (h) {
	case Health::Fault: return fault_mult;
	case Health::Degraded: return degraded_mult;
	case Health::Ok: return 1.0f;
	default: return 1.0f;
	}
}

bool ShaftKF::setpointChecksActive(hrt_abstime now, const TakeoffSample &tko) const
{
	if (!_params.use_takeoff_gate) {
		return true;
	}

	if (!tko.valid) {
		return _params.allow_checks_without_takeoff;
	}

	if (tko.state != static_cast<uint8_t>(_params.takeoff_state_flight)) {
		return false;
	}

	if (tko.first_flight_time == 0) {
		return true;
	}

	return (now > tko.first_flight_time)
	       && ((now - tko.first_flight_time) * 1.0e-6f >= _params.enable_after_flight_delay);
}

void ShaftKF::finishInfo(UpdateInfo &info, float z, const Vector<float, STATE_DIM> &H, float R)
{
	// info.timestamp = hrt_absolute_time();
	info.timestamp_sample = _last_predict_us;
	info.sequence = ++_seq;
	info.measurement = z;
	info.prediction = H.dot(_x);
	info.r = R;
	info.h0 = H(0);
	info.h1 = H(1);

	for (int i=0; i<STATE_DIM; i++) {
		info.state_after(i)=_x(i);
		info.p_diag_after(i)=_P(i,i);
	}

	info.p_v_diag_after(0) = _P(PX,VX);
	info.p_v_diag_after(1) = _P(PY,VY);
	info.p_v_diag_after(2) = _P(PZ,VZ);
}

bool ShaftKF::updateScalar(float z, const Vector<float, STATE_DIM> &H, float R, float nis_gate, UpdateInfo &info)
{
	R = math::max(R, _params.min_R);
	const float z_pred = H.dot(_x);
	float innov = z - z_pred;
	const float S = (H.transpose() * _P * H)(0,0) + R;

	if (fabsf(H(DYAW)) > 0.f) {
		innov = matrix::wrap_pi(innov);
	}

	info.innovation = innov;
	info.innovation_var = S;
	info.r = R;
	// info.H = H;
	info.h0 = H(0);
	info.h1 = H(1);
	info.measurement = z;
	info.prediction = z_pred;
	info.nis = PX4_ISFINITE(S) && S > _params.min_R ? (innov * innov) / S : NAN;

	if (!PX4_ISFINITE(info.nis) || info.nis > nis_gate) {
		info.accepted = false;
		info.health = info.health == Health::Unknown ? Health::Rejected : info.health;
		info.check_flags |= CHECK_NIS;
		finishInfo(info, z, H, R);
		accountEvent(info);
		return false;
	}

	const Vector<float, STATE_DIM> K = (_P * H) / S;
	info.k = K;
	_x += K * innov;

	// Joseph form: P = (I-KH) P (I-KH)' + K R K'
	SquareMatrix<float, STATE_DIM> I;
	I.setIdentity();
	SquareMatrix<float, STATE_DIM> KH;
	KH.zero();

	for (int r = 0; r < STATE_DIM; r++) {
		for (int c = 0; c < STATE_DIM; c++) {
			KH(r,c) = K(r) * H(c);
		}
	}

	const SquareMatrix<float, STATE_DIM> A = I - KH;
	SquareMatrix<float, STATE_DIM> KRKt;
	KRKt.zero();

	for (int r = 0; r < STATE_DIM; r++) {
		for (int c = 0; c < STATE_DIM; c++) {
			KRKt(r,c) = K(r) * R * K(c);
		}
	}

	_P = A * _P * A.transpose() + KRKt;
	symmetrizeP();
	info.accepted = true;
	finishInfo(info, z, H, R);
	accountEvent(info);
	return true;
}

void ShaftKF::accountEvent(const UpdateInfo &i)
{
	_status.event_count++;
	_status.update_count++;
	const uint16_t bit = 1u << static_cast<uint8_t>(i.event);
	_status.event_mask |= bit;
	if (i.accepted) {
		_status.accepted_update_count++;
		_status.accepted_event_mask |= bit;
	} else {
		_status.rejected_update_count++;
		_status.rejected_event_mask |= bit;
	}

	if (i.health == Health::Degraded) _status.degraded_event_mask |= bit;
	if (i.health == Health::Fault) _status.fault_event_mask |= bit;

	// if (PX4_ISFINITE(i.nis)) {
	// 	_status.max_nis = PX4_ISFINITE(_status.max_nis) ? math::max(_status.max_nis, i.nis) : i.nis;
	// 	_status.mean_nis = PX4_ISFINITE(_status.mean_nis) ?
	// 		((_status.mean_nis*_status.nis_count + i.nis)/(_status.nis_count+1)) : i.nis; _status.nis_count++;
	// }

	if (PX4_ISFINITE(i.innovation))
		_status.max_abs_innovation = PX4_ISFINITE(_status.max_abs_innovation) ?
			math::max(_status.max_abs_innovation, fabsf(i.innovation)) : fabsf(i.innovation);

	switch (i.event) {
	case EventType::VL_X:
		_status.vl_x_update_count++;
		// _status.last_vl_x_nis=i.nis;
		// _status.last_vl_x_check_flags=i.check_flags;
		// _status.vl_x_health=i.health;
		if(!i.accepted) _status.vl_x_reject_count++;
		if(i.health==Health::Degraded) _status.vl_x_degraded_count++;
		break;
	case EventType::VL_Y:
		_status.vl_y_update_count++;
		// _status.last_vl_y_nis=i.nis;
		// _status.last_vl_y_check_flags=i.check_flags;
		// _status.vl_y_health=i.health;
		if(!i.accepted) _status.vl_y_reject_count++;
		if(i.health==Health::Degraded) _status.vl_y_degraded_count++;
		break;
	case EventType::PAT_Z:
		_status.pat_z_update_count++;
		// _status.last_pat_z_nis=i.nis;
		// _status.last_pat_check_flags=i.check_flags;
		// _status.pat_health=i.health;
		if(!i.accepted) _status.pat_z_reject_count++;
		if(i.health==Health::Degraded) _status.pat_z_degraded_count++;
		break;
	case EventType::PAT_DYAW:
		_status.pat_dyaw_update_count++;
		// _status.last_pat_dyaw_nis=i.nis;
		if(!i.accepted) _status.pat_dyaw_reject_count++;
		if(i.health==Health::Degraded) _status.pat_dyaw_degraded_count++;
		break;
	case EventType::BARO_Z:
		_status.baro_z_update_count++;
		// _status.last_baro_z_nis=i.nis;
		// _status.last_baro_check_flags=i.check_flags;
		// _status.baro_health=i.health;
		if(!i.accepted) _status.baro_z_reject_count++;
		if(i.health==Health::Degraded) _status.baro_z_degraded_count++;
		break;
	case EventType::XY_ZUPT_X:
	case EventType::XY_ZUPT_Y:
		_status.xy_zupt_update_count++;
		if(!i.accepted) _status.xy_zupt_reject_count++;
		break;
	default:
		break;
	}
}

ShaftKF::UpdateInfo ShaftKF::fuseVlX(const VLSample &s, const AttitudeSample &a, const SetpointSample &sp, const TakeoffSample &tko)
{
	return fuseVLBody(s, EventType::VL_X, true, a, sp, tko);
}

ShaftKF::UpdateInfo ShaftKF::fuseVlY(const VLSample &s, const AttitudeSample &a, const SetpointSample &sp, const TakeoffSample &tko)
{
	return fuseVLBody(s, EventType::VL_Y, false, a, sp, tko);
}

ShaftKF::UpdateInfo ShaftKF::fuseVLBody(const VLSample &s, EventType event, bool bx, const AttitudeSample &att, const SetpointSample &sp, const TakeoffSample &tko)
{
	UpdateInfo info{};
	info.event = event;

	if (!_params.vl_enabled) {
		info.health = Health::Disabled;
		return info;
	}

	predictTo(s.timestamp_sample);

	const uint8_t offset_mm = bx ? _params.vl_offset_x_mm : _params.vl_offset_y_mm;
	const float sign = bx ? _params.vl_sign_body_x : _params.vl_sign_body_y;

	// Centered distance is converted to body-frame displacement by sign_body_x/y.
	float z_body = sign * (static_cast<float>(offset_mm - s.distance_mm) * 1.0e-3f);

	if (bx) {
		z_body += _params.vl_trolley_z_offset * tanf(att.pitch);
	} else {
		z_body -= _params.vl_trolley_z_offset * tanf(att.roll);
	}

	float yaw = att.yaw;

	if (_params.vl_use_dyaw_in_rotation) {
		yaw = matrix::wrap_pi(yaw + _x(DYAW));
	}

	const float c = cosf(yaw);
	const float sn = sinf(yaw);

	const float pred_body_x = c * _x(PX) + sn * _x(PY);
	const float pred_body_y = -sn * _x(PX) + c * _x(PY);
	const float pred_body_vx = c * _x(VX) + sn * _x(VY);
	const float pred_body_vy = -sn * _x(VX) + c * _x(VY);

	Vector<float, STATE_DIM> H;
	H.zero();

	float pred_other_body = NAN;
	float sigma_lever = 0.0f;
	float sigma_delay = 0.0f;

	if (bx) {
		H(PX) = c;
		H(PY) = sn;

		if (_params.vl_use_dyaw_in_rotation) {
			H(DYAW) = -sn * _x(PX) + c * _x(PY);
		}

		pred_other_body = pred_body_y;
		sigma_lever = fabsf(_params.vl_trolley_z_offset) * _params.vl_sigma_pitch;
		sigma_delay = fabsf(pred_body_vx) * _params.vl_time_skew_sigma;

	} else {
		H(PX) = -sn;
		H(PY) = c;

		if (_params.vl_use_dyaw_in_rotation) {
			H(DYAW) = -c * _x(PX) - sn * _x(PY);
		}

		pred_other_body = pred_body_x;
		sigma_lever = fabsf(_params.vl_trolley_z_offset) * _params.vl_sigma_roll;
		sigma_delay = fabsf(pred_body_vy) * _params.vl_time_skew_sigma;
	}

	float sigma = math::max(_params.vl_sigma_floor, fabsf(safeFinite(s.sigma_mm, 0.0f)) * 1.0e-3f);

	if (s.signal_per_spad_kcps < _params.vl_signal_fault) {
		sigma *= _params.vl_weak_signal_mult;
	}

	float R = sq(sigma) + sq(sigma_lever) + sq(sigma_delay);

	bool degraded = false;
	bool fault = false;
	uint16_t flags = 0;

	AxisHistory &hist = bx ? _vl_x_hist : _vl_y_hist;
	float raw_v = NAN;
	float raw_a = NAN;

	if (hist.valid && s.timestamp_sample > hist.last_t) {
		const float dt = (s.timestamp_sample - hist.last_t) * 1.0e-6f;

		if (dt > 1.0e-4f) {
			raw_v = (z_body - hist.last_z) / dt;
			raw_a = PX4_ISFINITE(hist.last_v) ? (raw_v - hist.last_v) / dt : NAN;
		}
	}

	if (_params.vl_health_enabled) {
		if (_params.vl_use_range_status && s.range_status != _params.vl_good_range_status) {
			fault = true;
			flags |= CHECK_RANGE_STATUS;
		}

		if (_params.vl_use_sigma) {
			if (s.sigma_mm > _params.vl_sigma_fault_mm) {
				fault = true;
				flags |= CHECK_HIGH_SIGMA;

			} else if (s.sigma_mm > _params.vl_sigma_degraded_mm) {
				degraded = true;
				flags |= CHECK_HIGH_SIGMA;
			}
		}

		if (_params.vl_use_signal_spad) {
			if (s.signal_per_spad_kcps < _params.vl_signal_fault) {
				fault = true;
				flags |= CHECK_LOW_SIGNAL;

			} else if (s.signal_per_spad_kcps < _params.vl_signal_degraded) {
				degraded = true;
				flags |= CHECK_LOW_SIGNAL;
			}
		}

		if (_params.vl_use_ambient_spad) {
			if (s.ambient_per_spad_kcps > _params.vl_ambient_fault) {
				fault = true;
				flags |= CHECK_HIGH_AMBIENT;

			} else if (s.ambient_per_spad_kcps > _params.vl_ambient_degraded) {
				degraded = true;
				flags |= CHECK_HIGH_AMBIENT;
			}
		}

		if (_params.vl_use_raw_velocity && PX4_ISFINITE(raw_v)) {
			if (fabsf(raw_v) > _params.vl_raw_v_fault) {
				fault = true;
				flags |= CHECK_RAW_VELOCITY;

			} else if (fabsf(raw_v) > _params.vl_raw_v_degraded) {
				degraded = true;
				flags |= CHECK_RAW_VELOCITY;
			}
		}

		if (_params.vl_use_raw_accel && PX4_ISFINITE(raw_a)) {
			if (fabsf(raw_a) > _params.vl_raw_a_fault) {
				fault = true;
				flags |= CHECK_RAW_ACCELERATION;

			} else if (fabsf(raw_a) > _params.vl_raw_a_degraded) {
				degraded = true;
				flags |= CHECK_RAW_ACCELERATION;
			}
		}

		if (_params.vl_use_radial_limit) {
			const float radius = hypotf(z_body, pred_other_body);
			// info.vl_radius_after_update = radius;	FIXME

			if (radius > _params.vl_radial_fault) {
				fault = true;
				flags |= CHECK_RADIAL_LIMIT;

			} else if (radius > _params.vl_radial_degraded) {
				degraded = true;
				flags |= CHECK_RADIAL_LIMIT;
			}
		}

		if (_params.vl_use_setpoint && sp.valid && setpointChecksActive(s.timestamp_sample, tko) && PX4_ISFINITE(raw_v)) {
			const float sp_vx = safeFinite(sp.vx, 0.0f);
			const float sp_vy = safeFinite(sp.vy, 0.0f);
			const float vsp = bx ? (c * sp_vx + sn * sp_vy) : (-sn * sp_vx + c * sp_vy);
			const float diff = fabsf(raw_v - vsp);

			if (diff > _params.vl_vxy_sp_fault_diff) {
				fault = true;
				flags |= CHECK_SETPOINT_MISMATCH;

			} else if (diff > _params.vl_vxy_sp_degraded_diff) {
				degraded = true;
				flags |= CHECK_SETPOINT_MISMATCH;
			}
		}
	}

	hist.valid = true;
	hist.last_z = z_body;
	hist.last_v = raw_v;
	hist.last_t = s.timestamp_sample;

	// info.vl_raw_v = raw_v;
	// info.vl_raw_a = raw_a;
	info.check_flags = flags;
	info.health = combineHealth(fault, degraded);

	R *= healthRFactor(info.health, _params.vl_degraded_R_mult, _params.vl_fault_R_mult);

	if (fault && _params.vl_reject_on_fault) {
		info.health = Health::Fault;
		finishInfo(info, z_body, H, R);
		accountEvent(info);
		return info;
	}

	updateScalar(z_body, H, R, _params.vl_nis_gate, info);
	finishInfo(info, z_body, H, R);	// FIXME czy to ma tutaj być
	accountEvent(info);
	return info;
}

ShaftKF::UpdateInfo ShaftKF::fusePAT(const PATSample &s, const AttitudeSample &, const SetpointSample &sp, const TakeoffSample &tko)
{
	UpdateInfo info{};
	info.event = EventType::PAT_Z;

	if (!_params.pat_enabled) {
		info.health = Health::Disabled;
		return info;
	}

	predictTo(s.timestamp_sample);

	const float mx = (_params.pat_use_resolution_cpi && s.x_resolution_cpi > 1.0f) ?
			 0.0254f / static_cast<float>(s.x_resolution_cpi) :
			 _params.pat_m_per_count_x;

	const float my = (_params.pat_use_resolution_cpi && s.y_resolution_cpi > 1.0f) ?
			 0.0254f / static_cast<float>(s.y_resolution_cpi) :
			 _params.pat_m_per_count_y;

	const float px = s.x_sum * mx;
	const float py = s.y_sum * my;
	const float ca = cosf(_params.pat_axis_angle);
	const float sa = sinf(_params.pat_axis_angle);

	const float vertical_m = _params.pat_sign_z * (sa * px + ca * py);
	const float tangential_m = ca * px - sa * py;

	// info.pat_vertical_m = vertical_m;
	// info.pat_tangential_m = tangential_m;

	float raw_vz = NAN;
	// float raw_yaw_rate = NAN;

	if (_last_pat_time && s.timestamp_sample > _last_pat_time) {
		const float dt = (s.timestamp_sample - _last_pat_time) * 1.0e-6f;

		if (dt > 1.0e-4f) {
			raw_vz = (vertical_m - _last_pat_vertical) / dt;

			// const float yaw_meas = matrix::wrap_pi(
			// 		       _params.yaw_initial + _params.yaw_sign * tangential_m / math::max(_params.yaw_rope_radius, 1.0e-4f));

			// raw_yaw_rate = matrix::wrap_pi(yaw_meas - _x(DYAW)) / dt;
		}
	}

	_last_pat_vertical = vertical_m;
	_last_pat_tangential = tangential_m;
	_last_pat_time = s.timestamp_sample;

	if (s.new_data) {
		_last_pat_new_data_time = s.timestamp_sample;
	}

	// info.pat_raw_vz = raw_vz;
	// info.pat_raw_yaw_rate = raw_yaw_rate;

	bool degraded = false;
	bool fault = false;
	uint16_t flags = 0;

	const bool quality_ok = s.squal >= _params.pat_squal_min;

	if (_params.pat_health_enabled) {
		if (!s.new_data && _params.pat_no_new_data_degraded) {
			degraded = true;
		}

		if (_params.pat_use_squal2) {
			if (s.squal2 < _params.pat_squal2_fault) {
				fault = true;
				flags |= CHECK_SQUAL2;

			} else if (s.squal2 < _params.pat_squal2_degraded) {
				degraded = true;
				flags |= CHECK_SQUAL2;
			}
		}

		if (_params.pat_use_shutter) {
			if (s.shutter > _params.pat_shutter_fault) {
				fault = true;
				flags |= CHECK_SHUTTER;

			} else if (s.shutter > _params.pat_shutter_degraded) {
				degraded = true;
				flags |= CHECK_SHUTTER;
			}
		}

		if (_params.pat_use_raw_vz && PX4_ISFINITE(raw_vz)) {
			if (fabsf(raw_vz) > _params.pat_raw_vz_fault) {
				fault = true;
				flags |= CHECK_RAW_VELOCITY;

			} else if (fabsf(raw_vz) > _params.pat_raw_vz_degraded) {
				degraded = true;
				flags |= CHECK_RAW_VELOCITY;
			}
		}

		if (_params.pat_use_setpoint
		    && sp.valid
		    && setpointChecksActive(s.timestamp_sample, tko)
		    && PX4_ISFINITE(sp.vz)
		    && PX4_ISFINITE(raw_vz)) {

			if (fabsf(sp.vz) > _params.pat_freeze_expected_vz && fabsf(raw_vz) < _params.pat_freeze_raw_vz) {
				fault = true;
				flags |= CHECK_FREEZE;
			}

			const float d = fabsf(raw_vz - sp.vz);

			if (d > _params.pat_vz_sp_fault_diff) {
				fault = true;
				flags |= CHECK_SETPOINT_MISMATCH;

			} else if (d > _params.pat_vz_sp_degraded_diff) {
				degraded = true;
				flags |= CHECK_SETPOINT_MISMATCH;
			}
		}

		if (_last_pat_new_data_time
		    && (s.timestamp_sample - _last_pat_new_data_time) * 1.0e-6f > _params.pat_comm_timeout) {
			fault = true;
			flags |= CHECK_TIMEOUT;
		}
	}

	_pat_health = combineHealth(fault, degraded);
	const float R_factor = healthRFactor(_pat_health, _params.pat_degraded_R_mult, _params.pat_fault_R_mult);

	info.health = _pat_health;
	info.check_flags = flags;

	if (fault && _params.pat_reject_on_fault) {
		Vector<float, STATE_DIM> H;
		H.zero();
		H(PZ) = 1.0f;

		finishInfo(info, vertical_m, H, sq(_params.pat_sigma_z0) * R_factor);
		accountEvent(info);
		return info;
	}

	if (s.new_data && quality_ok) {
		float sig = _params.pat_sigma_z0;

		sig *= sqrtf(static_cast<float>(_params.pat_squal_ref) / static_cast<uint8_t>(math::max(s.squal, _params.pat_squal_min)));


		const float Rz = (sq(sig)
				  + sq(_params.pat_scale_sigma_rel * fabsf(vertical_m))
				  + sq(fabsf(_x(VZ)) * _params.pat_time_skew_sigma)) * R_factor;

		Vector<float, STATE_DIM> H;
		H.zero();
		H(PZ) = 1.0f;

		updateScalar(vertical_m, H, Rz, _params.pat_nis_gate_z, info);
		finishInfo(info, vertical_m, H, Rz);
		accountEvent(info);

		if (_params.yaw_enabled) {
			UpdateInfo yi = info;
			yi.event = EventType::PAT_DYAW;

			const float dyaw = matrix::wrap_pi(
						   _params.yaw_initial + _params.yaw_sign * tangential_m / math::max(_params.yaw_rope_radius, 1.0e-4f));

			float syaw = _params.yaw_sigma_floor;

			syaw *= sqrtf(static_cast<float>(_params.yaw_squal_ref) / static_cast<uint8_t>(math::max(s.squal, _params.yaw_squal_min)));

			Vector<float, STATE_DIM> Hy;
			Hy.zero();
			Hy(DYAW) = 1.0f;

			updateScalar(dyaw,
				     Hy,
				     sq(math::max(syaw, _params.yaw_sigma_floor)) * R_factor,
				     _params.yaw_nis_gate,
				     yi);

			info.accepted = info.accepted || yi.accepted;
		}
	} else if (!s.new_data
		   && _params.pat_enable_zero_vz
		   && !fault
		   && _last_pat_new_data_time
		   && (s.timestamp_sample - _last_pat_new_data_time) * 1.0e-6f <= _params.pat_no_new_zupt_timeout) {

		Vector<float, STATE_DIM> H;
		H.zero();
		H(VZ) = 1.0f;

		updateScalar(0.0f, H, sq(_params.pat_sigma_vz0) * R_factor, _params.pat_nis_gate_vz0, info);
		finishInfo(info, 0.0f, H, sq(_params.pat_sigma_vz0) * R_factor);
		accountEvent(info);
	}

	return info;
}

ShaftKF::UpdateInfo ShaftKF::fuseBaro(const BaroSample &s)
{
	UpdateInfo info{};
	info.event = EventType::BARO_Z;

	if (!_params.baro_enabled || !PX4_ISFINITE(s.baro_alt_meter)) {
		info.health = Health::Disabled;
		return info;
	}

	predictTo(s.timestamp_sample);
	// FIXME Wziąć poprawny pomiar referencyjny - baro_bias?
	if (!_baro_ref_valid) {
		_baro_alt0 = PX4_ISFINITE(_params.baro_alt0) ? _params.baro_alt0 : s.baro_alt_meter;
		_baro_ref_valid = true;
	}

	float z = s.baro_alt_meter - _baro_alt0;

	if (_params.baro_z_positive_down) {
		z = -z;
	}

	Vector<float, STATE_DIM> H;
	H.zero();
	H(PZ) = 1.0f;

	updateScalar(z, H, sq(_params.baro_sigma_z), _params.baro_nis_gate, info);
	finishInfo(info, z, H, sq(_params.baro_sigma_z));
	accountEvent(info);
	return info;
}

ShaftKF::UpdateInfo ShaftKF::maybeFuseXyZupt(hrt_abstime now, const AttitudeSample &att, const SetpointSample &sp, const TakeoffSample &tko)
{
	UpdateInfo info{};
	info.event = EventType::XY_ZUPT_X;
	info.health = Health::Ok;

	if (!_params.xy_zupt_enabled || !sp.valid || !setpointChecksActive(now, tko)) {
		info.health = Health::Disabled;
		return info;
	}

	if (_last_xy_zupt && (now - _last_xy_zupt) * 1.0e-6f < _params.xy_zupt_min_interval) {
		return info;
	}

	const float vxy_sp = hypotf(safeFinite(sp.vx, 0.0f), safeFinite(sp.vy, 0.0f));
	const float tilt = hypotf(att.roll, att.pitch);

	if (vxy_sp > _params.xy_zupt_max_vxy_sp || tilt > _params.xy_zupt_max_tilt) {
		return info;
	}

	predictTo(now);

	Vector<float, STATE_DIM> Hx;
	Hx.zero();
	Hx(VX) = 1.0f;

	UpdateInfo ix = info;
	ix.event = EventType::XY_ZUPT_X;
	updateScalar(0.0f, Hx, sq(_params.xy_zupt_sigma_vxy0), _params.xy_zupt_nis_gate, ix);
	finishInfo(ix, 0.0f, Hx, sq(_params.xy_zupt_sigma_vxy0));
	accountEvent(ix);

	Vector<float, STATE_DIM> Hy;
	Hy.zero();
	Hy(VY) = 1.0f;

	UpdateInfo iy = info;
	iy.event = EventType::XY_ZUPT_Y;
	updateScalar(0.0f, Hy, sq(_params.xy_zupt_sigma_vxy0), _params.xy_zupt_nis_gate, iy);
	finishInfo(iy, 0.0f, Hy, sq(_params.xy_zupt_sigma_vxy0));
	accountEvent(iy);

	if (ix.accepted || iy.accepted) {
		_last_xy_zupt = now;
	}

	return iy.accepted ? iy : ix;
}
