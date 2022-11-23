# vehicle_msgs/AcousticModemBITRATE.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

string direction
int32 bitrate

diagnostic_msgs/KeyValue[] info         # optional field for additional data