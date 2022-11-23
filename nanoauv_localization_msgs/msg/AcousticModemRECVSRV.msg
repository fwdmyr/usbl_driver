# vehicle_msgs/AcousticModemRECVSRV.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

uint8 dest
uint8 source
string service
uint8 decoded
uint8 transmitted
float32 rssi
float32 integrity

diagnostic_msgs/KeyValue[] info         # optional field for additional data