// Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and limitations under the License.
//
// Author: Felix Widmayer (f.widmayer@irt.rwth-aachen.de)
//
#ifndef EVOLOGICS_DRIVER_EVOLOGICS_PARSER_HPP
#define EVOLOGICS_DRIVER_EVOLOGICS_PARSER_HPP

// standard library
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
// ros
#include <ros/ros.h>
// evologics_driver
#include <evologics_messages.hpp>
#include <evologics_utils.hpp>

namespace evologics
{
    class EvologicsParser
    {
    public:
        EvologicsParser();
        EvologicsParser(EvologicsParser &&) = default;
        EvologicsParser &operator=(EvologicsParser &&) = default;
        EvologicsParser(const EvologicsParser &) = default;
        EvologicsParser &operator=(const EvologicsParser &) = default;
        ~EvologicsParser() = default;

        /**
         * Initializes the parser. Must be called after instantiating a device object
         * @param nh ROS node handle
         * @param node_name ROS node name
         * @param msg_counter shared_ptr to MsgCounter object
         * @param last_ack shared_ptr to ACK object
         * @return Truth value of initialization success
         */
        bool initialize(ros::NodeHandle &nh, const std::string &node_name, std::shared_ptr<evologics::MsgCounter> msg_counter, std::shared_ptr<ACK> last_ack);

        /**
         * Redirects tokens to the message handlers based on identifier token
         * @param tokens Tokenized line
         */
        void parse(const std::vector<std::string> &tokens);

        /**
         * Publishes modem status with pre-defined frequency
         */
        void send_status();

        /**
         * Returns the next eligible polling command
         * @return Polling command
         */
        Command poll_modem();

    private:

        /**
         * Looks up msg_id in publisher map and publishes msg
         * @tparam M message type
         * @param msg_id message identifier
         * @param msg message object
         */
        template <typename M>
        void publish_message(const std::string &msg_id, const M &msg);

        /**
         * Instantiates publisher for message of type M and adds it to map with msg_id as key
         * @tparam M message type
         * @param msg_id message identifier
         */
        template <typename M>
        void setup_publisher(const std::string &msg_id);

        /**
         * Gets types of polling signals from ROS parameter server and pushes them into a vector
         * @return Number of different polling signals obtained from ROS parameter server
         */
        uint16_t init_poll_scheduler();

        /**
         * Converts modem_status and battery_level to the status message type and publishes the message
         * @param modem_status Status enum of the modem
         * @param battery_level Battery level
         */
        void handle_status(uint16_t modem_status, float battery_level);

        /*
         * Handlers that convert the token vector to the required message type and publishes the message
         */
        void handle_failed(const std::vector<std::string> &);
        void handle_delivered(const std::vector<std::string> &);
        void handle_acoustic_bm(const std::vector<std::string> &);
        void handle_acoustic_im(const std::vector<std::string> &);
        void handle_usbllong(const std::vector<std::string> &);
        void handle_usblangles(const std::vector<std::string> &);
        void handle_usblphyd(const std::vector<std::string> &);
        void handle_usblphyp(const std::vector<std::string> &);
        void handle_bitrate(const std::vector<std::string> &);
        void handle_srclevel(const std::vector<std::string> &);
        void handle_statusext(const std::vector<std::string> &);
        void handle_recvfailed(const std::vector<std::string> &);
        void handle_recvend(const std::vector<std::string> &);
        void handle_recvsrv(const std::vector<std::string> &);
        void handle_sendstart(const std::vector<std::string> &);
        void handle_sendend(const std::vector<std::string> &);
        void handle_raddr(const std::vector<std::string> &);
        void handle_response(const std::vector<std::string> &);
        void handle_batterylevel(const std::vector<std::string> &);       // AT?BV returns float (voltage)
        void handle_multipathstructure(const std::vector<std::string> &); // AT?P returns 8 lines of two ints ([timeline integrity]x8)
        void handle_relativevelocity(const std::vector<std::string> &);   // AT?V returns float (velocity)
        void handle_soundspeed(const std::vector<std::string> &);         // AT?CA returns int (soundspeed)

        ros::NodeHandle nh_;
        ros::Duration status_delay_;
        ros::Time last_status_time_;
        std::map<std::string, std::shared_ptr<ros::Publisher>> pub_map_;
        std::string node_name_;
        uint16_t modem_status_;
        std::vector<evologics::PollSchedule> poll_scheduler_;
        uint16_t poll_indexer_;
        uint16_t num_poll_signals_;
        bool poll_busy_flag_;
        ros::Time last_poll_time_;
        ros::Duration poll_timeout_duration_;
        float battery_level_;
        std::vector<std::vector<uint32_t>> multipath_structure_;
        uint16_t multipath_indexer_;
        std::shared_ptr<evologics::MsgCounter> msg_counter_;
        std::shared_ptr<ACK> last_ack_;
    };

} // namespace evologics

#endif // EVOLOGICS_DRIVER_EVOLOGICS_PARSER_HPP