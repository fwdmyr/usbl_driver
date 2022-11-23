# vehicle_msgs/AcousticModemUSBLPHYD.msg
# message containing USBL data published by modem driver

Header header                           # message header which contains current timestamp

float32 current_time                    # Timestamp of the string output measurement in the local devices System Time
float32 measurement_time                # Timestamp of the target position measurement in the local devices System Time
uint8 remote_address                    # the target's address

uint8 fix_type                          # Type of the fix (0: Position fix, 1: Angles fix)

float32 delay_15                        # Delay between transducers 1 and 5, in ns.
float32 delay_25                        # Delay between transducers 2 and 5, in ns.
float32 delay_35                        # Delay between transducers 3 and 5, in ns.
float32 delay_45                        # Delay between transducers 4 and 5, in ns.
float32 delay_12                        # Delay between transducers 1 and 2, in ns.
float32 delay_41                        # Delay between transducers 4 and 1, in ns.
float32 delay_32                        # Delay between transducers 3 and 2, in ns.
float32 delay_34                        # Delay between transducers 3 and 4, in ns.

diagnostic_msgs/KeyValue[] info         # optional field for additional data