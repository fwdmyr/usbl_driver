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
#ifndef EVOLOGICS_DRIVER_EVOLOGICS_DEVICE_HPP
#define EVOLOGICS_DRIVER_EVOLOGICS_DEVICE_HPP

// standard library
#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <iostream>
#include <queue>
// boost
#include <boost/asio.hpp>
// ros
#include <ros/ros.h>
// evologics_driver
#include <evologics_messages.hpp>
#include <evologics_utils.hpp>

namespace evologics
{
    class ModemDriver
    {
    public:
        virtual bool read(std::vector<std::string> &) = 0;
        virtual bool write() = 0;

    protected:
        ModemDriver();
        ModemDriver(ModemDriver &&) = default;
        ModemDriver &operator=(ModemDriver &&) = default;
        ModemDriver(const ModemDriver &) = default;
        ModemDriver &operator=(const ModemDriver &) = default;
        virtual ~ModemDriver() = default;
        virtual bool initialize(ros::NodeHandle &, const std::string &, const std::shared_ptr<evologics::MsgCounter>, const std::shared_ptr<ACK>) = 0;
        virtual void open() = 0;
        virtual void close() = 0;

        /**
         * Evaluates if modem is in error state, i.e. io error number threshold exceeded
         * @return Truth value of modem error state
         */
        bool in_error_state() const;

        ros::NodeHandle nh_;
        std::string node_name_;
        std::string ip_;
        int port_;
        uint16_t modem_enabled_;
        uint16_t num_io_errors_;
        uint16_t max_num_io_errors_;
    };

    class TCPModemDriver : ModemDriver
    {
    public:
        TCPModemDriver();
        ~TCPModemDriver() = default;

        /**
         * Reads a full line from TCP socket, tokenizes it and pushes them into the tokens vector
         * @param tokens empty vector that will hold the tokenized line
         * @return Truth value of read success
         */
        bool read(std::vector<std::string> &tokens) override;

        /**
         * Tries to write the oldest line in buffer to TCP socket. Locks write if IM msg was written and awaits modem ACK
         * @return Truth value of write success
         */
        bool write() override;

        /**
         * Initializes the device. Must be called after instantiating a device object
         * @param nh ROS node handle
         * @param node_name ROS node name
         * @param msg_counter shared_ptr to MsgCounter object
         * @param last_ack shared_ptr to ACK object
         * @return
         */
        bool initialize(ros::NodeHandle &nh, const std::string &node_name, const std::shared_ptr<evologics::MsgCounter>msg_counter, const std::shared_ptr<ACK> last_ack) override;

        /**
         * Adds cmd to the end of the write queue
         */
        void add_to_write_buffer(Command &cmd);

        /**
         * Queries the modem enable status
         * @return modem enable status
         */
        uint16_t modem_enabled() const;

        /**
         * Queries if write is locked
         * @return Truth value of write locked
         */
        bool write_locked() const;

        /**
         * Sets write lock status
         * @param write_lock Lock/Unlock flag
         */
        void lock_write(bool write_lock);

        /**
         * Flushes write buffer
         */
        void clear_write_buffer();

    private:

        /**
         * Tries to open connection to the TCP socket
         */
        void open() override;

        /**
         * Closes connection to the TCP socket
         */
        void close() override;

        boost::asio::io_service io_;
        boost::asio::ip::tcp::socket socket_;
        boost::asio::streambuf read_buffer_;
        std::queue<Command> write_buffer_;
        std::shared_ptr<evologics::MsgCounter> msg_counter_;
        std::shared_ptr<ACK> last_ack_;
        bool write_lock_;
    };

} // namespace evologics

#endif // EVOLOGICS_DRIVER_EVOLOGICS_DEVICE_HPP