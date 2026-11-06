# One STS3215 telemetry sample
int32 id
int32 pos          # raw counts 0..4095 (4096/rev, 2048 = center)
int32 speed        # counts/s
int32 load         # signed output duty, ~per-mille of max (tendon tension proxy)
float32 voltage    # V
int32 temperature  # degC
int32 current      # raw units
bool moving
bool error         # no reply from servo this sample
int32 goal         # last commanded goal (counts), -1 if none
