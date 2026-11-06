std_msgs/Header header
ServoState[] servos
bool torque_enabled
bool simulated      # true when running against the simulated board (no hardware moves)
string tripped     # empty = OK; else reason (load/temp limit, estop). Goals rejected until rearm.
