/****************************************************************************
 *
 *   Copyright (c) 2022 Ladon Robotics. All rights reserved.
 *
 *   Based on work by the PX4 Development Team.
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

#ifndef WINGSAIL_ACTUATOR_HPP
#define WINGSAIL_ACTUATOR_HPP

#include <uORB/topics/wingsail_actuator.h>

class MavlinkStreamWingsailActuator : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamWingsailActuator(mavlink); }

    static constexpr const char *get_name_static() { return "WINGSAIL_ACTUATOR"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_WINGSAIL_ACTUATOR; }

    const char *get_name() const override { return MavlinkStreamWingsailActuator::get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return (_wing_act_sub.advertised() or _wing_act_fore_sub.advertised() or _wing_act_mizzen_sub.advertised()) ?
            MAVLINK_MSG_ID_WINGSAIL_ACTUATOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
    }

private:
    explicit MavlinkStreamWingsailActuator(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _wing_act_sub{ORB_ID(wingsail_actuator)};
    uORB::Subscription _wing_act_fore_sub{ORB_ID(forewing_actuator)};
    uORB::Subscription _wing_act_mizzen_sub{ORB_ID(mizzenwing_actuator)};

    void send_msg(wingsail_actuator_s &act)
    {
        mavlink_wingsail_actuator_t msg{};
        msg.target_sail     = act.target_sail;
        msg.sail_angle_type = act.sail_angle_type;
        msg.sail_angle      = act.sail_angle;
        msg.flap_active     = act.flap_active_map;
        for (uint8_t i = 0; i < 8; i++)
        {
            msg.flap_angle[i] = (msg.flap_active & (1 << i)) ? act.flap_angle[i] : NAN;
        }
        mavlink_msg_wingsail_actuator_send_struct(_mavlink->get_channel(), &msg);
    }

    bool send() override
    {
        wingsail_actuator_s act;
	    uint8_t cnt = 0;
        while (_wing_act_sub.update(&act))
        {
            send_msg(act);
            cnt++;
        }
        while (_wing_act_fore_sub.update(&act))
        {
            send_msg(act);
            cnt++;
        }
        while (_wing_act_mizzen_sub.update(&act))
        {
            send_msg(act);
            cnt++;
        }

	    if (cnt > 0) return true;
        return false;
    }

};

#endif // WINGSAIL_ACTUATOR_HPP
