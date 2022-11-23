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
#include <evologics_device.hpp>

namespace evologics
{
    ModemDriver::ModemDriver() : num_io_errors_(0),
                                 max_num_io_errors_(MODEM::MAX_IO_ERRORS),
                                 modem_enabled_(MODEM::MODEM_ENABLED),
                                 port_(0){ };

    bool ModemDriver::in_error_state() const
    {
        if (num_io_errors_ >= max_num_io_errors_)
        {
            ROS_ERROR_STREAM("ModemDriver: Max number of errors exceeded!");
            return true;
        }
        return false;
    }

    TCPModemDriver::TCPModemDriver() : ModemDriver(), io_(), socket_(io_), write_lock_(false){};

    bool TCPModemDriver::initialize(ros::NodeHandle &nh, const std::string &node_name, const std::shared_ptr<MsgCounter> msg_counter, const std::shared_ptr<ACK> last_ack)
    {
        nh_ = nh;
        node_name_ = node_name;
        msg_counter_ = msg_counter;
        last_ack_ = last_ack;
        if (!nh_.getParam(node_name_ + "/modem_config/tcp_config/ip", ip_))
        {
            ROS_ERROR_STREAM("TCPModemDriver unable to load IP from config.");
            return false;
        }
        if (!nh_.getParam(node_name_ + "/modem_config/tcp_config/port", port_))
        {
            ROS_ERROR_STREAM("TCPModemDriver unable to load port from config.");
            return false;
        }
        return true;
    }

    bool TCPModemDriver::write_locked() const {
        return write_lock_;
    }

    void TCPModemDriver::lock_write(bool write_lock) {
        write_lock_ = write_lock;
    };

    void TCPModemDriver::add_to_write_buffer(Command &cmd)
    {
        if (cmd.msg.back() != '\n')
            cmd.msg += '\n';
        write_buffer_.push(cmd);
    }

    void TCPModemDriver::clear_write_buffer()
    {
        std::queue<Command> empty;
        std::swap(write_buffer_, empty);
    }

    uint16_t TCPModemDriver::modem_enabled() const
    {
        return modem_enabled_;
    }

    void TCPModemDriver::open()
    {
        if (socket_.is_open())
            return;
        boost::system::error_code err;
        boost::asio::ip::tcp::endpoint recv_endpoint(boost::asio::ip::address::from_string(ip_, err), port_);
        socket_.connect(recv_endpoint);

        ROS_INFO_STREAM("TCPModemDriver socket open: " << socket_.is_open() << " | OS error = " << err.value());
    }

    void TCPModemDriver::close()
    {
        boost::system::error_code err;
        socket_.close(err);
        ROS_ERROR_STREAM("TCPModemDriver socket close error:  " << err.value());
        sleep(1.0);
    }

    bool TCPModemDriver::read(std::vector<std::string> &tokens)
    {
        if (in_error_state() || ros::isShuttingDown())
            return false;
        boost::system::error_code err;
        open();
        uint16_t bytes_available = socket_.available(err);
        if (bytes_available == 0) // make sure that read_until does not block the socket when no data
            return true;
        // read full line
        uint16_t bytes_received = boost::asio::read_until(socket_, read_buffer_, '\n', err);
        if (err.value() == boost::system::errc::success)
        {
            std::istream is(&read_buffer_);
            std::string read_str;
            std::getline(is, read_str);
            // tokenize line and fill empty tokens vector
            split(read_str, DELIMITER, std::back_inserter(tokens));
            (msg_counter_->recv)++;
            return true;
        }
        num_io_errors_++;
        ROS_ERROR_STREAM("TCPModemDriver read error: " << num_io_errors_ << " of " << max_num_io_errors_);
        close();
        return false;
    }

    bool TCPModemDriver::write()
    {
        if (in_error_state() || ros::isShuttingDown())
            return false;
        if (write_buffer_.empty())
            return true;
        boost::system::error_code err;
        open();
        const auto cmd = write_buffer_.front(); // FIFO buffer
        uint16_t bytes_sent = boost::asio::write(socket_, boost::asio::buffer(cmd.msg), err);
        if (err.value() == boost::system::errc::success)
        {
            if (cmd.type == CMDTYPE::IM) // lock the write and wait for instant message receive ack
                write_lock_ = true;
            *last_ack_ = cmd.ack;
            if (*last_ack_ == ACK::IM_ACK)
                (msg_counter_->sent_im_ack)++;
            if (*last_ack_ == ACK::BURST_ACK)
                (msg_counter_->sent_burst_ack)++;
            write_buffer_.pop();
            return true;
        }
        num_io_errors_++;
        ROS_ERROR_STREAM("TCPModemDriver write error: " << num_io_errors_ << " of " << max_num_io_errors_);
        close();
        return false;
    }

} // namespace evologics