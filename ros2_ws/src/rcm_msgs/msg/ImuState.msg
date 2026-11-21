# Tip IMU, filtered (6-axis Madgwick, no magnetometer -> yaw drifts slowly; roll/pitch/tilt don't)
std_msgs/Header header
string state                        # offline | calibrating | ok
string message                      # e.g. "hold the IMU still to calibrate the gyro"
float32 rate_hz
geometry_msgs/Quaternion orientation   # sensor -> world, world z up
geometry_msgs/Quaternion relative      # rotation since reference pose (identity if none)
bool has_reference
float64 roll_deg
float64 pitch_deg
float64 tilt_from_ref_deg           # angle between gravity directions now vs reference (yaw-free); NaN if no reference
geometry_msgs/Vector3 accel         # m/s^2, sensor frame (raw)
geometry_msgs/Vector3 gyro          # rad/s, sensor frame, bias-corrected
geometry_msgs/Vector3 gyro_bias     # rad/s
float32 temperature
uint32 bad_lines
