# vehicle_msgs/AcousticModemMULTIPATHSTRUCTURE.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

uint32[] first
uint32[] second
uint32[] third
uint32[] fourth
uint32[] fifth
uint32[] sixth
uint32[] seventh
uint32[] eigth

diagnostic_msgs/KeyValue[] info         # optional field for additional data
