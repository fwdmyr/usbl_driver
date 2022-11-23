# vehicle_msgs/AcousticModemSRCLEVEL.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

uint8 source_level

diagnostic_msgs/KeyValue[] info         # optional field for additional data