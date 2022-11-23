# vehicle_msgs/AcousticModemUSBLPHYP.msg
# message containing USBL data published by modem driver

Header header                           # message header which contains current timestamp

float32 current_time                    # Timestamp of the string output measurement in the local devices System Time
float32 measurement_time                # Timestamp of the target position measurement in the local devices System Time
uint8 remote_address                    # the target's address

uint8 fix_type                          # Type of the fix (0: Position fix, 1: Angles fix)

float32 X123                            # Coordinates of the target, in m, floating point,
float32 Y123                            # estimated by an array of transducers 1, 2 and 3.
float32 Z123

float32 X432                            # Coordinates of the target, in m, floating point,
float32 Y432                            # estimated by an array of transducers 4, 3 and 2.
float32 Z432

float32 X341                            # Coordinates of the target, in m, floating point,
float32 Y341                            # estimated by an array of transducers 3, 4 and 1.
float32 Z341

float32 X412                            # Coordinates of the target, in m, floating point,
float32 Y412                            # estimated by an array of transducers 4, 1 and 2.
float32 Z412

float32 X153                            # Coordinates of the target, in m, floating point,
float32 Y153                            # estimated by an array of transducers 1, 5 and 3.
float32 Z153

float32 X254                            # Coordinates of the target, in m, floating point,
float32 Y254                            # estimated by an array of transducers 2, 5 and 4.
float32 Z254

diagnostic_msgs/KeyValue[] info         # optional field for additional data