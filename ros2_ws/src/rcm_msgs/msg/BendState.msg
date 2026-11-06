# CC configuration ESTIMATED FROM TENDON LENGTHS (servo encoders) — not a true shape
# measurement: it ignores tendon stretch and payload-induced deflection.
std_msgs/Header header
float64 theta              # [rad]
float64 phi                # [rad]
float64 kappa              # [1/m]
geometry_msgs/Point tip    # [m], base frame, z along straight backbone
float64[] tendon_dl_mm     # length change per tendon (negative = shortened)
float64 common_mode_mm     # mean dl: nonzero => slack/stretch vs calibration
float64 cmd_theta          # last commanded
float64 cmd_phi
