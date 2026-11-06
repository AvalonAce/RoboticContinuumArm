# Raw servo goal positions, sent as one simultaneous SyncWrite
int32[] ids
int32[] positions  # counts
uint16 speed       # counts/s (0 -> bridge default)
uint8 acc          # 0 -> bridge default
