std_msgs/Header header
ServoState[] servos
bool torque_enabled
string tripped     # empty = OK; else reason (load/temp limit, estop). Goals rejected until rearm.
