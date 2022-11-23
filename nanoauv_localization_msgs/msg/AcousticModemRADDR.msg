# vehicle_msgs/AcousticModemRADDR.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

uint8 remote_address

diagnostic_msgs/KeyValue[] info         # optional field for additional data