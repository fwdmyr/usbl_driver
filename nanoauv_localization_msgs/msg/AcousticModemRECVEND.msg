# vehicle_msgs/AcousticModemRECVEND.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

int32 timestamp
int32 duration
float32 rssi
float32 integrity

diagnostic_msgs/KeyValue[] info         # optional field for additional data