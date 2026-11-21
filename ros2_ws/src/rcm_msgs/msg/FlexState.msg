# Flex (bend) sensors along the arm, from the flex sensor board (flex_bridge, ~25 Hz).
std_msgs/Header header
string state            # ok | offline | sim
string message
float32 rate_hz         # board sample rate seen
string[] names          # e.g. "flex1 (A2)"
float32[] volts         # divider voltage (flat ~1.65 V with 10 kOhm; bending lowers it)
float32[] resistance_ohm
float32[] angle_deg     # bend angle from the flat/90 deg calibration (linear in resistance)
bool[] calibrated       # false = 90 deg point still the default guess (R90 = 2 x R_flat)
