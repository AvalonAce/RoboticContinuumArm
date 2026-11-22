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
float32[] from_cm       # sensor span along the bendable length, measured from the base
float32[] to_cm
float32 total_deg       # whole-arm bend estimated from all sensors (overlaps averaged, not double-counted)
float32 covered_cm      # how much of the bendable length the sensors cover
float32 length_cm       # bendable length
