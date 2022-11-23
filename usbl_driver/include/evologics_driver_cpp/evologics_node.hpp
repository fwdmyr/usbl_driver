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
#ifndef EVOLOGICS_DRIVER_EVOLOGICS_NODE_HPP
#define EVOLOGICS_DRIVER_EVOLOGICS_NODE_HPP

// standard library
#include <mutex>
#include <condition_variable>
#include <thread>
#include <sstream>
#include <chrono>
// ros
#include <ros/ros.h>
// evologics_driver
#include <evologics_device.hpp>
#include <evologics_parser.hpp>
#include <evologics_utils.hpp>

namespace evologics
{

    constexpr float TIMER_PERIOD = 0.1;

    class EvologicsNode
    {
    public:
        explicit EvologicsNode(ros::NodeHandle &);
        EvologicsNode(EvologicsNode &&) = default;
        EvologicsNode &operator=(EvologicsNode &&) = default;
        EvologicsNode(const EvologicsNode &) = default;
        EvologicsNode &operator=(const EvologicsNode &) = default;
        ~EvologicsNode() = default;

    private:

        /**
         * Wraps the service loop and attaches it to a ROS timer and reinitializes device and parser on WD timeout
         * @param event Parameterizes the ROS timer callback
         */
        void service_loop_cb(const ros::TimerEvent &event);

        /**
         * Handles writing polling cmds to the socket as well as reading and parsing lines
         */
        void service_loop();

        /**
         * Callback that sends instant message to other modem after receiving instant message from it
         * @param msg AcousticModemPayload received from other modem
         */
        void im_callback(const nanoauv_localization_msgs::AcousticModemPayload &msg);

        /**
         * Callback that sends burst message to other modem after receiving burst message from it
         * @param msg AcousticModemPayload received from other modem
         */
        void burst_callback(const nanoauv_localization_msgs::AcousticModemPayload &msg);

        /**
         * Initializes the modem by enabling extended notifications and performing some housekeeping
         * @return Truth value of init success
         */
        bool init_modem();

        ros::NodeHandle nh_;
        ros::Timer timer_;
        ros::Subscriber im_sub_;
        ros::Subscriber burst_sub_;
        evologics::TCPModemDriver device_;
        evologics::EvologicsParser parser_;
        std::string node_name_;
        float read_rate_;
        float watchdog_timeout_;
        std::shared_ptr<MsgCounter> msg_counter_;
        std::shared_ptr<ACK> last_ack_;
    };

} // namespace evologics

#endif // EVOLOGICS_DRIVER_EVOLOGICS_NODE_HPP