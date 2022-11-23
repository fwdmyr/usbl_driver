# vehicle_msgs/AcousticModemRECVFAILED.msg
# message containing data published by modem driver

Header header                           # message header which contains current timestamp

float32 velocity
float32 rssi
float32 integrity

diagnostic_msgs/KeyValue[] info         # optional field for additional data