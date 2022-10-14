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

#ifndef WIND_DATA_HPP
#define WIND_DATA_HPP

#include <uORB/topics/wind_data.h>

class MavlinkStreamWindData : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamWindData(mavlink); }

    static constexpr const char *get_name_static() { return "WIND_DATA"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_WIND_DATA; }

    const char *get_name() const override { return MavlinkStreamWindData::get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return _wind_data_sub.advertised() ?
            MAVLINK_MSG_ID_WIND_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
    }
private:
    explicit MavlinkStreamWindData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _wind_data_sub{ORB_ID(wind_data)};

    bool send() override
    {
        wind_data_s wind;
        if (_wind_data_sub.update(&wind))
        {
            mavlink_wind_data_t msg{};
            msg.source_sail = wind.sail_position;
            msg.wind_type = WIND_DATA_TYPE_RAW;
            msg.speed = wind.raw_speed;
            msg.direction = wind.raw_dir;
            mavlink_msg_wind_data_send_struct(_mavlink->get_channel(), &msg);

            if (wind.app_valid)
            {
                msg.wind_type = WIND_DATA_TYPE_APPARENT;
                msg.speed = wind.app_speed;
                msg.direction = wind.app_dir;
                mavlink_msg_wind_data_send_struct(_mavlink->get_channel(), &msg);
            }

            if (wind.true_valid)
            {
                msg.wind_type = WIND_DATA_TYPE_TRUE;
                msg.speed = wind.true_speed;
                msg.direction = wind.true_dir;
                mavlink_msg_wind_data_send_struct(_mavlink->get_channel(), &msg);
            }

            return true;
        }

        return false;
    }

};

#endif // WIND_DATA_HPP
