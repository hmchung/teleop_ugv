#!bin/bash
rosservice call /head_imu/set_stream_rate 0 20 1
rostopic echo /head_imu/imu/data
exec "$@"
