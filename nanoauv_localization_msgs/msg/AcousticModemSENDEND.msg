# vehicle_msgs/AcousticModemSENDEND.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

uint8 dest
string type
uint32 timestamp
uint32 duration

diagnostic_msgs/KeyValue[] info         # optional field for additional data