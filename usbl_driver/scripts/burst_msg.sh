#!/usr/bin/env bash

# usage
print_usage() {
    echo "Usage: $(basename $0) <address> <payload>"
    echo "Send the <payload> to acoustic modem with address <address>."
    echo ""
    echo "Mandatory arguments:"
    echo "  <modem_node>: ROS node name of modem as string"
    echo "  <address>: unsigned int from range (0, 255)"
    echo "  <payload>: a string not longer 1024 chars"
    echo ""
}

# script body
if [[ ! -n $1 ]]; then
    print_usage
    exit 1
fi

# vars
MODEM_NODE=$1
ADDRESS=$2
PAYLOAD=$3

echo ""
echo "Sending burst message..."
echo "With payload: 	 	${PAYLOAD}"
echo "Using ROS node:  ${MODEM_NODE}"
echo "To remote address: 	${ADDRESS}"
echo ""

rostopic pub -1 /${MODEM_NODE}/burst/out nanoauv_localization_msgs/AcousticModemPayload "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
msg_id: 0
address: ${ADDRESS}
ack: false
bitrate: 0
rssi: 0.0
integrity: 0.0
propagation_time: 0
duration: 0
relative_velocity: 0.0
payload: '${PAYLOAD}'
info:
- {key: '', value: ''}"


