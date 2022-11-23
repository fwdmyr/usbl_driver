# vehicle_msgs/AcousticModemSTATUSEXT.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

string status
string status_parameter

diagnostic_msgs/KeyValue[] info         # optional field for additional data