# State of the task sequencer's sequence engine
uint8 IDLE=0
uint8 RUNNING=1
uint8 DONE=2
uint8 FAILED=3
uint8 PAUSED=4      # abort latched
uint8 CANCELLED=5   # e-stop or operator cancel

string sequence     # name of the active (or last finished) sequence
uint8 state         # state from the constants above
uint32 step_index   # 0-based index of the step in progress
uint32 step_total   # total steps for sequence completion
string detail       # description of the current step, or the failure/stop reason
