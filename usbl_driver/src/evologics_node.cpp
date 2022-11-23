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

// evologics_driver
#include <evologics_node.hpp>

namespace evologics
{
    using namespace std::chrono_literals;

    EvologicsNode::EvologicsNode(ros::NodeHandle &nh) : device_(),
                                                        parser_()
    {
        nh_ = nh;
        node_name_ = ros::this_node::getName();
        msg_counter_ = std::make_shared<MsgCounter>();
        last_ack_ = std::make_shared<ACK>(ACK::BURST_ACK);
        read_rate_ = 0.0;
        watchdog_timeout_ = 10.0;
        device_.initialize(nh_, node_name_, msg_counter_, last_ack_);
        parser_.initialize(nh_, node_name_, msg_counter_, last_ack_);
        if (!nh_.getParam(node_name_ + "/modem_config/read_rate", read_rate_))
            ROS_ERROR_STREAM("TCPModemDriver unable to load IP from config.");
        if (!init_modem())
            ROS_ERROR_STREAM("TCPModemDriver modem initialization failed.");
        timer_ = nh_.createTimer(ros::Duration(1.0 / (read_rate_)), &EvologicsNode::service_loop_cb, this);
        im_sub_ = nh_.subscribe(node_name_ + "/im/out", 1, &EvologicsNode::im_callback, this);
        burst_sub_ = nh_.subscribe(node_name_ + "/burst/out", 1, &EvologicsNode::burst_callback, this);
        nh_.getParam(node_name_ + "/modem_config/watchdog_timeout", watchdog_timeout_);
    }

    bool EvologicsNode::init_modem()
    {
        auto now = ros::Time::now();
        auto init_start_time = now;
        Command cmd = {"AT@ZX1", ACK::NO_ACK, CMDTYPE::MODE}; // Enable extended notifications cmd
        while (now - init_start_time <= ros::Duration(10.0))
        {
            device_.add_to_write_buffer(cmd);
            device_.write();
            std::vector<std::string> tokens;
            device_.read(tokens);
            std::string msg_type = tokens.empty() ? "" : tokens[0];
            if (msg_type.find("OK") != std::string::npos)
            {
                ROS_INFO_STREAM("[" << node_name_ << "]: Extended notifications enabled");
                device_.clear_write_buffer();
                return true;
            }
            now = ros::Time::now();
            std::this_thread::sleep_for(0.1s);
        }
        device_.clear_write_buffer();
        return false;
    }

    void EvologicsNode::service_loop()
    {
        auto cmd = parser_.poll_modem();
        if (cmd.ack != ACK::SKIP_ACK) { // Polling is not busy
            if (nh_.param<bool>(node_name_ + "/modem_config/verbose_flag", false))
                ROS_INFO_STREAM("Sending request " << cmd.msg << " to modem");
            device_.add_to_write_buffer(cmd); // Add polling request to write buffer
        }
        if (!device_.write_locked())
            device_.write(); // Write oldest cmd in buffer
        std::vector<std::string> tokens;
        device_.read(tokens); // Read most recent line from socket
        if (nh_.param<bool>(node_name_ + "/modem_config/verbose_flag", false))
        {
            print_tokens(node_name_, tokens);
        }
        std::string msg_type = tokens.empty() ? "" : tokens[0];
        if (msg_type == "DELIVEREDIM" || msg_type == "FAILEDIM" || msg_type == "CANCELLEDIM")
            device_.lock_write(false); // Unlock write after receiving IM msg ACK
        parser_.parse(tokens); // Parse most recent line from socket
        parser_.send_status();
    }

    void EvologicsNode::service_loop_cb(const ros::TimerEvent &event)
    {
        // wraps the service loop and reinitializes parser and handler
        // if the service loop times out
        std::mutex m;
        std::condition_variable cv;

        std::thread t([&cv, this]()
        {
            service_loop();
            cv.notify_one();
        });

        t.detach();

        {
            std::unique_lock<std::mutex> l(m);
            // WD timeout
            if (cv.wait_for(l, std::chrono::duration<float>(watchdog_timeout_)) == std::cv_status::timeout)
            {
                ROS_WARN_STREAM("[" << node_name_ << "]: Watchdog timeout detected");
                ROS_WARN_STREAM("[" << node_name_ << "]: Reinitializing...");
                msg_counter_ = std::make_shared<MsgCounter>();
                last_ack_ = std::make_shared<ACK>(ACK::BURST_ACK);
                device_.initialize(nh_, node_name_, msg_counter_, last_ack_);
                parser_.initialize(nh_, node_name_, msg_counter_, last_ack_);
                ROS_WARN_STREAM("[" << node_name_ << "]: Done");
            }
        }
    }

    void EvologicsNode::im_callback(const nanoauv_localization_msgs::AcousticModemPayload &ros_msg)
    {
        // callback for incoming instant messages that sends instant message to the source modem
        uint16_t msg_len = ros_msg.payload.size();
        if (msg_len > MSG::IM_SIZE)
        {
            ROS_WARN_STREAM("Received too long IM message with size " << msg_len << " / " << MSG::IM_SIZE);
            return;
        }
        std::stringstream ss;
        ss << "AT*SENDIM" << ',' << msg_len << ',' << uint(ros_msg.address) << ',';
        if (!ros_msg.ack)
            ss << "no";
        ss << "ack" << ',' << ros_msg.payload;
        std::string msg = ss.str();
        auto ack = (ros_msg.ack) ? ACK::IM_ACK : ACK::NO_ACK;
        Command cmd = {msg, ack, CMDTYPE::IM};
        if (device_.modem_enabled())
        {
            device_.add_to_write_buffer(cmd);
        }
    }

    void EvologicsNode::burst_callback(const nanoauv_localization_msgs::AcousticModemPayload &ros_msg)
    {
        // callback for incoming burst messages that sends burst message to the source modem
        uint16_t msg_len = ros_msg.payload.size();
        if (msg_len > MSG::BURST_SIZE)
        {
            ROS_WARN_STREAM("Received too long BURST message with size " << msg_len << " / " << MSG::BURST_SIZE);
            return;
        }
        std::stringstream ss;
        ss << "AT*SEND" << ',' << msg_len << ',' << uint(ros_msg.address) << ',' << ros_msg.payload;
        std::string msg = ss.str();
        auto ack = ACK::BURST_ACK;
        Command cmd = {msg, ack, CMDTYPE::BURST};
        if (device_.modem_enabled())
        {
            device_.add_to_write_buffer(cmd);
        }
    }
} // namespace evologics