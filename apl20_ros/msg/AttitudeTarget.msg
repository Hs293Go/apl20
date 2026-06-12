# Attitude- or body-rate-level setpoint with collective thrust, mirroring
# mavros_msgs/AttitudeTarget so a MAVROS user can drive the autopilot's
# ~/setpoint_raw/attitude topic with the familiar fields. It enters the cascade
# below the position loop:
#   - type_mask & IGNORE_ATTITUDE == 0  -> `orientation` (body->NED quaternion)
#     feeds the attitude controller (attitude-control mode).
#   - type_mask & IGNORE_ATTITUDE != 0  -> `body_rate` [rad/s] feeds the rate
#     controller directly (acro / rate-control mode), bypassing attitude.
# `thrust` is the normalized collective in [0, 1] (PX4 convention), always used
# unless IGNORE_THRUST is set.
std_msgs/Header header

uint8 type_mask
uint8 IGNORE_ROLL_RATE = 1
uint8 IGNORE_PITCH_RATE = 2
uint8 IGNORE_YAW_RATE = 4
uint8 IGNORE_THRUST = 64
uint8 IGNORE_ATTITUDE = 128

geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 body_rate
float64 thrust
