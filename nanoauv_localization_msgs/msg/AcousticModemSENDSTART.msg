# vehicle_msgs/AcousticModemSENDSTART.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

uint8 dest
string type
uint32 duration
uint32 delay

diagnostic_msgs/KeyValue[] info         # optional field for additional data