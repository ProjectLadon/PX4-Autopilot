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

#ifndef PID_TUNE_HPP
#define PID_TUNE_HPP

#include <uORB/topics/pid_tune.h>

class MavlinkStreamPidTune : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamPidTune(mavlink); }

    static constexpr const char *get_name_static() { return "PID_TUNE"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_PID_TUNING; }

    const char *get_name() const override { return MavlinkStreamPidTune::get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return _sub.advertised() ?
            MAVLINK_MSG_ID_PID_TUNING + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
    }
private:
    explicit MavlinkStreamPidTune(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _sub{ORB_ID(pid_tune)};

    bool send() override
    {
        pid_tune_s tune;
        if (_sub.update(&tune))
        {
            mavlink_pid_tuning_t msg{};
            msg.axis            = tune.axis;
            msg.desired         = tune.desired;
            msg.achieved        = tune.achieved;
            msg.FF              = tune.ff;
            msg.P               = tune.p;
            msg.I               = tune.i;
            msg.D               = tune.d;
            msg.SRate           = tune.srate;
            msg.PDmod           = tune.pdmod;
            mavlink_msg_pid_tuning_send_struct(_mavlink->get_channel(), &msg);
            return true;
        }
        return false;
    }
};

#endif // PID_TUNE_HPP
