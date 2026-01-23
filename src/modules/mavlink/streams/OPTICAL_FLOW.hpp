/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef OPTICAL_FLOW_HPP
#define OPTICAL_FLOW_HPP

#include <uORB/topics/optical_navigation_vertical.h>

class MavlinkStreamOpticalFlow : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOpticalFlow(mavlink); }

	static constexpr const char *get_name_static() { return "OPTICAL_FLOW"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_OPTICAL_FLOW; }

	const char *get_name() const override { return MavlinkStreamOpticalFlow::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _optical_navigation_vertical.advertised() ? (MAVLINK_MSG_ID_OPTICAL_FLOW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) :
		       0;
	}

private:
	explicit MavlinkStreamOpticalFlow(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _optical_navigation_vertical{ORB_ID(optical_navigation_vertical)};

	bool send() override
	{
		optical_navigation_vertical_s nav;

		if (_optical_navigation_vertical.update(&nav)) {
			mavlink_optical_flow_t msg{};

			msg.time_usec = nav.timestamp;
			msg.sensor_id = nav.sensor_id;
			msg.flow_x = (int16_t)nav.x_sum;
			msg.flow_y = (int16_t)nav.y_sum;
			msg.flow_comp_m_x = nav.z_m;
			msg.flow_comp_m_y = nav.vz_m_s;
			msg.quality = nav.squal;

			// if (PX4_ISFINITE(flow.distance_m)) {
			// 	msg.ground_distance = flow.distance_m;

			// } else {
				msg.ground_distance = (float)nav.shutter;
			// }

			msg.flow_rate_x = nav.var_z_m2;
			msg.flow_rate_y = nav.var_vz_m2s2;

			mavlink_msg_optical_flow_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // OPTICAL_FLOW_RAD_HPP
