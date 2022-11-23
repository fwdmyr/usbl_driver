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
#ifndef EVOLOGICS_DRIVER_EVOLOGICS_MESSAGES_HPP
#define EVOLOGICS_DRIVER_EVOLOGICS_MESSAGES_HPP

// ros
#include <ros/ros.h>
// nanoauv_localization_msgs
#include <nanoauv_localization_msgs/AcousticModemPayload.h>
#include <nanoauv_localization_msgs/AcousticModemUSBLLONG.h>
#include <nanoauv_localization_msgs/AcousticModemUSBLANGLES.h>
#include <nanoauv_localization_msgs/AcousticModemUSBLPHYD.h>
#include <nanoauv_localization_msgs/AcousticModemUSBLPHYP.h>
#include <nanoauv_localization_msgs/AcousticModemBITRATE.h>
#include <nanoauv_localization_msgs/AcousticModemSRCLEVEL.h>
#include <nanoauv_localization_msgs/AcousticModemSTATUSEXT.h>
#include <nanoauv_localization_msgs/AcousticModemRECVFAILED.h>
#include <nanoauv_localization_msgs/AcousticModemRECVEND.h>
#include <nanoauv_localization_msgs/AcousticModemRECVSRV.h>
#include <nanoauv_localization_msgs/AcousticModemSENDSTART.h>
#include <nanoauv_localization_msgs/AcousticModemSENDEND.h>
#include <nanoauv_localization_msgs/AcousticModemRADDR.h>
#include <nanoauv_localization_msgs/AcousticModemAck.h>
#include <nanoauv_localization_msgs/AcousticModemStatus.h>
#include <nanoauv_localization_msgs/AcousticModemRELATIVEVELOCITY.h>
#include <nanoauv_localization_msgs/AcousticModemSOUNDSPEED.h>
#include <nanoauv_localization_msgs/AcousticModemMULTIPATHSTRUCTURE.h>
// diagnostic_msgs
#include <diagnostic_msgs/KeyValue.h>

namespace evologics
{

    constexpr char DELIMITER = ',';
    constexpr uint16_t QUEUE_SIZE = 10;

    namespace MULTIPATH
    {
        constexpr uint16_t TIMELINE = 0;
        constexpr uint16_t INTEGRITY = 1;
        constexpr uint8_t MATRIX_ROWS = 8;
        constexpr uint8_t MATRIX_COLUMNS = 2;
    } // namespace multipath

    namespace MODEM
    {
        constexpr uint16_t MODEM_ENABLED = 1;
        constexpr uint16_t MODEM_DISABLED = 0;
        constexpr uint16_t ERROR = 1;
        constexpr uint16_t OK = 0;
        constexpr uint16_t TIMEOUT_PROP_READ = 5;
        constexpr uint16_t MAX_IO_ERRORS = 5;
        constexpr float DEFAULT_POLLING_INTERVAL = 60.0;
    } // namespace modem

    namespace MSG
    {
        constexpr uint16_t IM_SIZE = 64;
        constexpr uint16_t BURST_SIZE = 1024;
        constexpr uint16_t RECV_SIZE = 10;
        constexpr uint16_t RECVIM_SIZE = 10;
        constexpr uint16_t RECVIMS_SIZE = 10;
        constexpr uint16_t RECVPBM_SIZE = 9;
        constexpr uint16_t USBLLONG_SIZE = 17;
        constexpr uint16_t USBLANGLES_SIZE = 14;
        constexpr uint16_t USBLPHYD_SIZE = 13;
        constexpr uint16_t USBLPHYP_SIZE = 23;
        constexpr uint16_t BITRATE_SIZE = 3;
        constexpr uint16_t SRCLEVEL_SIZE = 2;
        constexpr uint16_t STATUSEXT_SIZE = 3;
        constexpr uint16_t RECVFAILED_SIZE = 4;
        constexpr uint16_t RECVEND_SIZE = 5;
        constexpr uint16_t RECVSRV_SIZE = 8;
        constexpr uint16_t SENDSTART_SIZE = 5;
        constexpr uint16_t SENDEND_SIZE = 5;
        constexpr uint16_t RADDR_SIZE = 2;
        constexpr uint16_t FAILED_SIZE = 3;
        constexpr uint16_t DELIVERED_SIZE = 3;
    } // namespace msg

    enum class ACK
    {
        NO_ACK,
        IM_ACK,
        BURST_ACK,
        SKIP_ACK
    }; // namespace ack

    enum class CMDTYPE
    {
        IM,
        BURST,
        POLL,
        MODE
    }; // namespace cmdtype

    struct PollSchedule
    {
        std::string polling_cmd;
        ros::Duration polling_interval;
        ros::Time last_polling_time;
    };

    struct Command
    {
        std::string msg;
        ACK ack;
        CMDTYPE type;
    };

    struct MsgCounter
    {
        MsgCounter() : recv_im_ack(0), sent_im_ack(0), recv_burst_ack(0), sent_burst_ack(0), recv(0) {}
        uint16_t recv_im_ack;
        uint16_t sent_im_ack;
        uint16_t recv_burst_ack;
        uint16_t sent_burst_ack;
        uint16_t recv;
    };

} // namespace evologics

#endif // EVOLOGICS_DRIVER_EVOLOGICS_MESSAGES_HPP