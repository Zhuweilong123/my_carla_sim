std_msgs/Header header
float64 x
float64 y
float64 yaw
float64 vx
float64 vy
float64 yaw_rate
float64 steering_angle
float64 acceleration
# FIFO after this physics step, oldest pending steering command first.
# Shares header.stamp with the measured angle; valid even while safety braking.
bool steering_history_valid
float64[] steering_delay_queue
