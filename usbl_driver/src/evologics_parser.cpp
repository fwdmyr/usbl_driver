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
#include <evologics_parser.hpp>

namespace evologics
{

    EvologicsParser::EvologicsParser() : pub_map_(){};

    bool EvologicsParser::initialize(ros::NodeHandle &nh, const std::string &node_name, const std::shared_ptr<MsgCounter> msg_counter, const std::shared_ptr<ACK> last_ack)
    {
        nh_ = nh;
        node_name_ = node_name;
        modem_status_ = MODEM::MODEM_ENABLED;
        battery_level_ = 0.0;
        multipath_indexer_ = 0;
        multipath_structure_ = std::vector<std::vector<uint32_t>>(MULTIPATH::MATRIX_ROWS, std::vector<uint32_t>(MULTIPATH::MATRIX_COLUMNS, 0));
        msg_counter_ = msg_counter;
        last_ack_ = last_ack;
        float status_rate;
        if (!nh_.getParam(node_name_ + "/modem_config/status_rate", status_rate))
        {
            ROS_ERROR_STREAM("EvologicsParser unable to load status publish rate from config.");
            return false;
        }
        status_delay_ = ros::Duration(1.0 / status_rate);
        last_status_time_ = ros::Time::now();
        last_poll_time_ = ros::Time::now();
        poll_timeout_duration_ = ros::Duration(nh_.param<float>(node_name_ + "/modem_config/poll_timeout_duration", 10.0));
        num_poll_signals_ = init_poll_scheduler();
        poll_indexer_ = 0;
        poll_busy_flag_ = false;
        return true;
    }

    uint16_t EvologicsParser::init_poll_scheduler()
    {
        // determine which info will be polled from the modem by accessing the ROS parameter server
        ros::Time now = ros::Time::now();
        if (nh_.param<bool>(node_name_ + "/poll_config/poll_batterylevel", false))
        {
            poll_scheduler_.push_back(PollSchedule{"AT?BV", ros::Duration(nh_.param<float>(node_name_ + "/poll_config/interval_batterylevel", MODEM::DEFAULT_POLLING_INTERVAL)), now});
        }
        if (nh_.param<bool>(node_name_ + "/poll_config/poll_relativevelocity", false))
        {
            poll_scheduler_.push_back(PollSchedule{"AT?V", ros::Duration(nh_.param<float>(node_name_ + "/poll_config/interval_relativevelocity", MODEM::DEFAULT_POLLING_INTERVAL)), now});
        }
        if (nh_.param<bool>(node_name_ + "/poll_config/poll_soundspeed", false))
        {
            poll_scheduler_.push_back(PollSchedule{"AT?CA", ros::Duration(nh_.param<float>(node_name_ + "/poll_config/interval_soundspeed", MODEM::DEFAULT_POLLING_INTERVAL)), now});
        }
        if (nh_.param<bool>(node_name_ + "/poll_config/poll_multipathstructure", false))
        {
            poll_scheduler_.push_back(PollSchedule{"AT?P", ros::Duration(nh_.param<float>(node_name_ + "/poll_config/interval_multipathstructure", MODEM::DEFAULT_POLLING_INTERVAL)), now});
        }
        return poll_scheduler_.size();
    }

    Command EvologicsParser::poll_modem()
    {
        ros::Time now = ros::Time::now();
        Command cmd{"", ACK::SKIP_ACK, CMDTYPE::POLL}; // device will ignore all commands that contain SKIP_ACK
        if (poll_busy_flag_ && (now - last_poll_time_ <= poll_timeout_duration_))
            return cmd;
        // Find the next eligible poll cmd by checking if enough time has passed since same cmd was sent the last time
        // Given available poll cmds [A B C]
        // If last cmd was A, check B -> C -> A
        // If last cmd was B, check C -> A -> B
        // If last cmd was C, check A -> B -> C
        // Avoids starving the poll cmds at the end of the list for high polling frequencies
        for (size_t i = 0; i < num_poll_signals_; ++i)
        {
            if (now - poll_scheduler_[poll_indexer_].last_polling_time >= poll_scheduler_[poll_indexer_].polling_interval)
            {
                // Build polling command
                cmd.msg = poll_scheduler_[poll_indexer_].polling_cmd;
                cmd.ack = ACK::NO_ACK;
                poll_busy_flag_ = true; // Block other polling commands from being added to write buffer of device
                last_poll_time_ = now;
                poll_scheduler_[poll_indexer_].last_polling_time = now;
                break;
            }
            poll_indexer_ = (poll_indexer_ >= num_poll_signals_ - 1) ? 0 : poll_indexer_ + 1; // increment index with roll-over
        }
        return cmd;
    }

    void EvologicsParser::parse(const std::vector<std::string> &tokens)
    {
        if (tokens.empty())
            return;

        std::string message_type = clean_message_type_token(tokens[0]); // this uniquely identifies each message type

        // if publishing for message type is enabled, call handler for message type
        if ((message_type == "RECV") && nh_.param<bool>(node_name_ + "/msg_config/publish_recv", false))
        {
            handle_acoustic_bm(tokens);
            return;
        }
        else if ((message_type.find("DELIVERED") != std::string::npos) && nh_.param<bool>(node_name_ + "/msg_config/publish_delivered", false))
        {
            handle_delivered(tokens);
            return;
        }
        else if ((message_type =="FAILED") && nh_.param<bool>(node_name_ + "/msg_config/publish_failed", false))
        {
            handle_failed(tokens);
            return;
        }
        else if ((message_type == "RECVIM") && nh_.param<bool>(node_name_ + "/msg_config/publish_recvim", false))
        {
            handle_acoustic_im(tokens);
            return;
        }
        else if ((message_type.find("USBLLONG") != std::string::npos) && nh_.param<bool>(node_name_ + "/msg_config/publish_usbllong", false))
        {
            handle_usbllong(tokens);
            return;
        }
        else if ((message_type.find("USBLANGLES") != std::string::npos) && nh_.param<bool>(node_name_ + "/msg_config/publish_usblangles", false))
        {
            handle_usblangles(tokens);
            return;
        }
        else if ((message_type == "USBLPHYD") && nh_.param<bool>(node_name_ + "/msg_config/publish_usblphyd", false))
        {
            handle_usblphyd(tokens);
            return;
        }
        else if ((message_type == "USBLPHYP") && nh_.param<bool>(node_name_ + "/msg_config/publish_usblphyp", false))
        {
            handle_usblphyp(tokens);
            return;
        }
        else if ((message_type == "BITRATE") && nh_.param<bool>(node_name_ + "/msg_config/publish_bitrate", false))
        {
            handle_bitrate(tokens);
            return;
        }
        else if ((message_type == "SRCLEVEL") && nh_.param<bool>(node_name_ + "/msg_config/publish_srclevel", false))
        {
            handle_srclevel(tokens);
            return;
        }
        else if ((message_type == "STATUS") && nh_.param<bool>(node_name_ + "/msg_config/publish_status", false))
        {
            handle_statusext(tokens);
            return;
        }
        else if ((message_type == "RECVFAILED") && nh_.param<bool>(node_name_ + "/msg_config/publish_recvfailed", false))
        {
            handle_recvfailed(tokens);
            return;
        }
        else if ((message_type == "RECVEND") && nh_.param<bool>(node_name_ + "/msg_config/publish_recvend", false))
        {
            handle_recvend(tokens);
            return;
        }
        else if ((message_type == "RECVSRV") && nh_.param<bool>(node_name_ + "/msg_config/publish_recvsrv", false))
        {
            handle_recvsrv(tokens);
            return;
        }
        else if ((message_type == "SENDSTART") && nh_.param<bool>(node_name_ + "/msg_config/publish_sendstart", false))
        {
            handle_sendstart(tokens);
            return;
        }
        else if ((message_type == "SENDEND") && nh_.param<bool>(node_name_ + "/msg_config/publish_sendend", false))
        {
            handle_sendend(tokens);
            return;
        }
        else if ((message_type == "RADDR") && nh_.param<bool>(node_name_ + "/msg_config/publish_raddr", false))
        {
            handle_raddr(tokens);
            return;
        }
        else if (is_polling_response(message_type)) // if first token does not match any message_type, check if tokens match response to poll cmd
        {
            handle_response(tokens);
            return;
        }
        else
        {
            return;
        }
    }

    template <typename M>
    void EvologicsParser::publish_message(const std::string &msg_id, const M &msg)
    {
        pub_map_[msg_id]->publish(msg);
    }

    template <typename M>
    void EvologicsParser::setup_publisher(const std::string &msg_id)
    {
        std::string topic = node_name_ + '/' + msg_id;
        pub_map_[msg_id] = std::make_shared<ros::Publisher>(nh_.advertise<M>(topic, QUEUE_SIZE));
    }

    void EvologicsParser::send_status()
    {
        ros::Time now = ros::Time::now();
        if (now - last_status_time_ >= status_delay_)
        {
            handle_status(modem_status_, battery_level_);
            last_status_time_ = now;
        }
    }

    void EvologicsParser::handle_status(uint16_t modem_status, float battery_level)
    {
        std::string msg_id = "status";
        auto msg = nanoauv_localization_msgs::AcousticModemStatus();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.status = (modem_status == MODEM::MODEM_ENABLED)
                             ? nanoauv_localization_msgs::AcousticModemStatus::MODEM_ENABLED
                             : nanoauv_localization_msgs::AcousticModemStatus::MODEM_DISABLED;
            msg.battery_level = battery_level;
            if (msg_counter_->sent_im_ack || msg_counter_->sent_burst_ack)
            {
                diagnostic_msgs::KeyValue im_success;
                im_success.key = "im_ack_success_rate";
                im_success.value = std::to_string(static_cast<float>(msg_counter_->recv_im_ack) / static_cast<float>(msg_counter_->sent_im_ack));
                diagnostic_msgs::KeyValue burst_success;
                burst_success.key = "burst_ack_success_rate";
                burst_success.value = std::to_string(static_cast<float>(msg_counter_->recv_burst_ack) / static_cast<float>(msg_counter_->sent_burst_ack));
                diagnostic_msgs::KeyValue total_sent;
                total_sent.key = "total_sent";
                total_sent.value = std::to_string(msg_counter_->sent_im_ack + msg_counter_->sent_burst_ack);
                diagnostic_msgs::KeyValue total_received;
                total_received.key = "total_received";
                total_received.value = std::to_string(msg_counter_->recv);
                std::vector<diagnostic_msgs::KeyValue> info{im_success, burst_success, total_sent, total_received};
                msg.info = info;
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemStatus>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemStatus>(msg_id, msg);
    }

    void EvologicsParser::handle_response(const std::vector<std::string> &tokens)
    {

        if (tokens[0].empty())
        {
            ROS_WARN_STREAM("Empty message received");
            return;
        }

        std::string active_poll_cmd = poll_scheduler_[poll_indexer_].polling_cmd;

        if (nh_.param<bool>(node_name_ + "/modem_config/verbose_flag", false))
            ROS_INFO_STREAM("Received response for request " << active_poll_cmd);

        // Second redirection step as polling responses do not possess unique identifiers
        // -> Check active poll command as only one polling response can be pending at each time
        if (active_poll_cmd == "AT?BV")
        {
            handle_batterylevel(tokens);
        }
        else if (active_poll_cmd == "AT?V")
        {
            handle_relativevelocity(tokens);
        }
        else if (active_poll_cmd == "AT?CA")
        {
            handle_soundspeed(tokens);
        }
        else if (active_poll_cmd == "AT?P")
        {
            handle_multipathstructure(tokens);
        }
        else
        {
            ROS_WARN_STREAM("Unidentified message");
        }
    }

    void EvologicsParser::handle_relativevelocity(const std::vector<std::string> &tokens)
    {
        poll_busy_flag_ = false;
        poll_indexer_ = (poll_indexer_ >= num_poll_signals_ - 1) ? 0 : poll_indexer_ + 1;
        std::string msg_id = "relativevelocity";
        auto msg = nanoauv_localization_msgs::AcousticModemRELATIVEVELOCITY();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.data = std::stof(tokens[0]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemRELATIVEVELOCITY>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemRELATIVEVELOCITY>(msg_id, msg);
    }

    void EvologicsParser::handle_soundspeed(const std::vector<std::string> &tokens)
    {
        poll_busy_flag_ = false;
        poll_indexer_ = (poll_indexer_ >= num_poll_signals_ - 1) ? 0 : poll_indexer_ + 1;
        std::string msg_id = "soundspeed";
        auto msg = nanoauv_localization_msgs::AcousticModemSOUNDSPEED();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.data = std::stof(tokens[0]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemSOUNDSPEED>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemSOUNDSPEED>(msg_id, msg);
    }

    void EvologicsParser::handle_batterylevel(const std::vector<std::string> &tokens)
    {
        poll_busy_flag_ = false;
        poll_indexer_ = (poll_indexer_ >= num_poll_signals_ - 1) ? 0 : poll_indexer_ + 1;
        battery_level_ = std::stof(tokens[0]);
    }

    void EvologicsParser::handle_multipathstructure(const std::vector<std::string> &tokens)
    {
        std::string multipath_line = tokens[0];
        std::vector<uint32_t> data;
        fill_vector_from_string<uint32_t>(multipath_line, data);
        // for (const auto & word : data) ROS_INFO_STREAM(word);
        multipath_structure_[multipath_indexer_][MULTIPATH::TIMELINE] = data[MULTIPATH::TIMELINE];
        multipath_structure_[multipath_indexer_][MULTIPATH::INTEGRITY] = data[MULTIPATH::INTEGRITY];
        if (multipath_indexer_ < MULTIPATH::MATRIX_ROWS - 1)
        {
            multipath_indexer_++;
            return;
        }
        poll_busy_flag_ = false;
        poll_indexer_ = (poll_indexer_ >= num_poll_signals_ - 1) ? 0 : poll_indexer_ + 1;
        multipath_indexer_ = 0;
        std::string msg_id = "multipathstructure";
        auto msg = nanoauv_localization_msgs::AcousticModemMULTIPATHSTRUCTURE();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.first = multipath_structure_[0];
            msg.second = multipath_structure_[1];
            msg.third = multipath_structure_[2];
            msg.fourth = multipath_structure_[3];
            msg.fifth = multipath_structure_[4];
            msg.sixth = multipath_structure_[5];
            msg.seventh = multipath_structure_[6];
            msg.eigth = multipath_structure_[7];
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemMULTIPATHSTRUCTURE>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemMULTIPATHSTRUCTURE>(msg_id, msg);
    }

    void EvologicsParser::handle_acoustic_bm(const std::vector<std::string> &tokens)
    {
        std::string msg_id = "burst/in";
        auto msg = nanoauv_localization_msgs::AcousticModemPayload();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.address = std::stoi(tokens[2]);
            msg.bitrate = std::stoi(tokens[4]);
            msg.rssi = std::stof(tokens[5]);
            msg.integrity = std::stof(tokens[6]);
            msg.propagation_time = std::stoi(tokens[7]);
            msg.relative_velocity = std::stof(tokens[8]);
            std::string payload = tokens[9];
            for (std::vector<std::string>::const_iterator it = tokens.begin() + 10; it != tokens.end(); ++it)
            {
                payload += ',' + *it;
            }
            payload.erase(std::remove(payload.begin(), payload.end(), '\0'), payload.end());
            payload.erase(std::remove(payload.begin(), payload.end(), '\r'), payload.end());
            msg.payload = payload;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemPayload>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemPayload>(msg_id, msg);
        (msg_counter_->sent_burst_ack)++;
    }

    void EvologicsParser::handle_acoustic_im(const std::vector<std::string> &tokens)
    {
        std::string msg_id = "im/in";
        auto msg = nanoauv_localization_msgs::AcousticModemPayload();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.address = std::stoi(tokens[2]);
            msg.duration = std::stoi(tokens[5]);
            msg.rssi = std::stof(tokens[6]);
            msg.integrity = std::stof(tokens[7]);
            msg.relative_velocity = std::stof(tokens[8]);
            std::string payload = tokens[9];
            for (std::vector<std::string>::const_iterator it = tokens.begin() + 10; it != tokens.end(); ++it)
            {
                payload += ',' + *it;
            }
            msg.payload = payload;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemPayload>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemPayload>(msg_id, msg);
        (msg_counter_->sent_im_ack)++;
    }

    void EvologicsParser::handle_usbllong(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::USBLLONG_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser USBLLONG Message bad format");
            return;
        }

        std::string msg_id = "usbllong";
        auto msg = nanoauv_localization_msgs::AcousticModemUSBLLONG();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.current_time = std::stof(tokens[1]);
            msg.measurement_time = std::stof(tokens[2]);
            msg.remote_address = std::stoi(tokens[3]);
            msg.X = std::stof(tokens[4]);
            msg.Y = std::stof(tokens[5]);
            msg.Z = std::stof(tokens[6]);
            msg.E = std::stof(tokens[7]);
            msg.N = std::stof(tokens[8]);
            msg.U = std::stof(tokens[9]);
            msg.roll = std::stof(tokens[10]);
            msg.pitch = std::stof(tokens[11]);
            msg.yaw = std::stof(tokens[12]);
            msg.propagation_time = std::stof(tokens[13]);
            msg.rssi = std::stof(tokens[14]);
            msg.integrity = std::stof(tokens[15]);
            msg.accuracy = std::stof(tokens[16]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemUSBLLONG>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemUSBLLONG>(msg_id, msg);
    }

    void EvologicsParser::handle_usblangles(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::USBLANGLES_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser USBLANGLES Message bad format");
            return;
        }

        std::string msg_id = "usblangles";
        auto msg = nanoauv_localization_msgs::AcousticModemUSBLANGLES();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.current_time = std::stof(tokens[1]);
            msg.measurement_time = std::stof(tokens[2]);
            msg.remote_address = std::stoi(tokens[3]);
            msg.lbearing = std::stof(tokens[4]);
            msg.lelevation = std::stof(tokens[5]);
            msg.bearing = std::stof(tokens[6]);
            msg.elevation = std::stof(tokens[7]);
            msg.roll = std::stof(tokens[8]);
            msg.pitch = std::stof(tokens[9]);
            msg.yaw = std::stof(tokens[10]);
            msg.rssi = std::stof(tokens[11]);
            msg.integrity = std::stof(tokens[12]);
            msg.accuracy = std::stof(tokens[13]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemUSBLANGLES>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemUSBLANGLES>(msg_id, msg);
    }

    void EvologicsParser::handle_usblphyd(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::USBLPHYD_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser USBLPHYD Message bad format");
            return;
        }
        std::string msg_id = "usblphyd";
        auto msg = nanoauv_localization_msgs::AcousticModemUSBLPHYD();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.current_time = std::stof(tokens[1]);
            msg.measurement_time = std::stof(tokens[2]);
            msg.remote_address = std::stoi(tokens[3]);
            msg.fix_type = std::stoi(tokens[4]);
            msg.delay_15 = std::stof(tokens[5]);
            msg.delay_25 = std::stof(tokens[6]);
            msg.delay_35 = std::stof(tokens[7]);
            msg.delay_45 = std::stof(tokens[8]);
            msg.delay_12 = std::stof(tokens[9]);
            msg.delay_41 = std::stof(tokens[10]);
            msg.delay_32 = std::stof(tokens[11]);
            msg.delay_32 = std::stof(tokens[12]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemUSBLPHYD>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemUSBLPHYD>(msg_id, msg);
    }

    void EvologicsParser::handle_usblphyp(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::USBLPHYP_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser USBLPHYP Message bad format");
            return;
        }
        std::string msg_id = "usblphyp";
        auto msg = nanoauv_localization_msgs::AcousticModemUSBLPHYP();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.current_time = std::stof(tokens[1]);
            msg.measurement_time = std::stof(tokens[2]);
            msg.remote_address = std::stoi(tokens[3]);
            msg.fix_type = std::stoi(tokens[4]);
            msg.X123 = std::stof(tokens[5]);
            msg.Y123 = std::stof(tokens[6]);
            msg.Z123 = std::stof(tokens[7]);
            msg.X432 = std::stof(tokens[8]);
            msg.Y432 = std::stof(tokens[9]);
            msg.Z432 = std::stof(tokens[10]);
            msg.X341 = std::stof(tokens[11]);
            msg.Y341 = std::stof(tokens[12]);
            msg.Z341 = std::stof(tokens[13]);
            msg.X412 = std::stof(tokens[14]);
            msg.Y412 = std::stof(tokens[15]);
            msg.Z412 = std::stof(tokens[16]);
            msg.X153 = std::stof(tokens[17]);
            msg.Y153 = std::stof(tokens[18]);
            msg.Z153 = std::stof(tokens[19]);
            msg.X254 = std::stof(tokens[20]);
            msg.Y254 = std::stof(tokens[21]);
            msg.Z254 = std::stof(tokens[22]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemUSBLPHYP>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemUSBLPHYP>(msg_id, msg);
    }

    void EvologicsParser::handle_bitrate(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::BITRATE_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser BITRATE Message bad format");
            return;
        }
        std::string msg_id = "bitrate";
        auto msg = nanoauv_localization_msgs::AcousticModemBITRATE();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.direction = tokens[1];
            msg.bitrate = std::stoi(tokens[2]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemBITRATE>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemBITRATE>(msg_id, msg);
    }

    void EvologicsParser::handle_srclevel(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::SRCLEVEL_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser SRCLEVEL Message bad format");
            return;
        }
        std::string msg_id = "srclevel";
        auto msg = nanoauv_localization_msgs::AcousticModemSRCLEVEL();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.source_level = std::stoi(tokens[1]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemSRCLEVEL>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemSRCLEVEL>(msg_id, msg);
    }

    void EvologicsParser::handle_statusext(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::STATUSEXT_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser STATUSEXT Message bad format");
            return;
        }
        std::string msg_id = "statusext";
        auto msg = nanoauv_localization_msgs::AcousticModemSTATUSEXT();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.status = tokens[1];
            msg.status_parameter = tokens[2];
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemSTATUSEXT>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemSTATUSEXT>(msg_id, msg);
    }

    void EvologicsParser::handle_recvfailed(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::RECVFAILED_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser RECVFAILED Message bad format");
            return;
        }
        std::string msg_id = "recvfailed";
        auto msg = nanoauv_localization_msgs::AcousticModemRECVFAILED();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.velocity = std::stof(tokens[1]);
            msg.rssi = std::stof(tokens[2]);
            msg.integrity = std::stof(tokens[3]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemRECVFAILED>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemRECVFAILED>(msg_id, msg);
    }

    void EvologicsParser::handle_recvend(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::RECVEND_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser RECVEND Message bad format");
            return;
        }
        std::string msg_id = "recvend";
        auto msg = nanoauv_localization_msgs::AcousticModemRECVEND();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.timestamp = std::stoi(tokens[1]);
            msg.duration = std::stoi(tokens[2]);
            msg.rssi = std::stof(tokens[3]);
            msg.integrity = std::stof(tokens[4]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemRECVEND>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemRECVEND>(msg_id, msg);
    }

    void EvologicsParser::handle_recvsrv(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::RECVSRV_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser RECVSRV Message bad format");
            return;
        }
        std::string msg_id = "recvsrv";
        auto msg = nanoauv_localization_msgs::AcousticModemRECVSRV();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.dest = std::stoi(tokens[1]);
            msg.source = std::stoi(tokens[2]);
            msg.service = tokens[3];
            msg.decoded = std::stoi(tokens[4]);
            msg.transmitted = std::stoi(tokens[5]);
            msg.rssi = std::stof(tokens[6]);
            msg.integrity = std::stof(tokens[7]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemRECVSRV>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemRECVSRV>(msg_id, msg);
    }

    void EvologicsParser::handle_sendstart(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::SENDSTART_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser SENDSTART Message bad format");
            return;
        }
        std::string msg_id = "sendstart";
        auto msg = nanoauv_localization_msgs::AcousticModemSENDSTART();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.dest = std::stoi(tokens[1]);
            msg.type = tokens[2];
            msg.duration = std::stoi(tokens[3]);
            msg.delay = std::stoi(tokens[4]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemSENDSTART>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemSENDSTART>(msg_id, msg);
    }

    void EvologicsParser::handle_sendend(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::SENDEND_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser SENDEND Message bad format");
            return;
        }
        std::string msg_id = "sendend";
        auto msg = nanoauv_localization_msgs::AcousticModemSENDEND();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.dest = std::stoi(tokens[1]);
            msg.type = tokens[2];
            msg.timestamp = std::stoi(tokens[3]);
            msg.duration = std::stoi(tokens[4]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemSENDEND>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemSENDEND>(msg_id, msg);
    }

    void EvologicsParser::handle_raddr(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::RADDR_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser RADDR Message bad format");
            return;
        }
        std::string msg_id = "raddr";
        auto msg = nanoauv_localization_msgs::AcousticModemRADDR();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.remote_address = std::stoi(tokens[1]);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemRADDR>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemRADDR>(msg_id, msg);
    }

    void EvologicsParser::handle_failed(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::FAILED_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser FAILED Message bad format");
            return;
        }
        std::string msg_id;
        if (*last_ack_ == ACK::IM_ACK)
        {
            msg_id = "im/ack";
        }
        else if (*last_ack_ == ACK::BURST_ACK)
        {
            msg_id = "burst/ack";
        }
        else if ((*last_ack_ == ACK::NO_ACK) || (*last_ack_ == ACK::SKIP_ACK))
        {
            return;
        }
        else
        {
            ROS_WARN_STREAM("EvologicsParser FAILED Message bad ack type");
            return;
        }
        auto msg = nanoauv_localization_msgs::AcousticModemAck();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.msg_id = std::stoi(tokens[1]);
            msg.ack = 0;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemAck>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemAck>(msg_id, msg);
    }

    void EvologicsParser::handle_delivered(const std::vector<std::string> &tokens)
    {
        if (tokens.size() != MSG::DELIVERED_SIZE)
        {
            ROS_WARN_STREAM("EvologicsParser DELIVERED Message bad format");
            return;
        }
        std::string msg_id;
        (msg_counter_->recv)++;
        if (*last_ack_ == ACK::IM_ACK)
        {
            msg_id = "im/ack";
            (msg_counter_->recv_im_ack)++;
        }
        else if (*last_ack_ == ACK::BURST_ACK)
        {
            msg_id = "burst/ack";
            (msg_counter_->recv_burst_ack)++;
        }
        else if ((*last_ack_ == ACK::NO_ACK) || (*last_ack_ == ACK::SKIP_ACK))
        {
            return;
        }
        else
        {
            ROS_WARN_STREAM("EvologicsParser DELIVERED Message bad ack type");
            return;
        }

        auto msg = nanoauv_localization_msgs::AcousticModemAck();

        try
        {
            msg.header.stamp = ros::Time::now();
            msg.msg_id = std::stoi(tokens[1]);
            msg.ack = static_cast<int>(*last_ack_);
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
        if (pub_map_.find(msg_id) == pub_map_.end())
        {
            setup_publisher<nanoauv_localization_msgs::AcousticModemAck>(msg_id);
        }
        publish_message<nanoauv_localization_msgs::AcousticModemAck>(msg_id, msg);
    }

} // namespace evologics