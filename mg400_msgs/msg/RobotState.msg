# State derived from realtime feedback, with a local ServoJ session overlay.
uint8 UNKNOWN=0
uint8 NOT_READY=1
uint8 MANUAL=2
uint8 DISABLED=3
uint8 ENABLED=4
uint8 RUNNING=5
uint8 PAUSED_OR_JOG=6
uint8 ERROR=7
uint8 SERVO=8

# Zero indicates that no realtime feedback has been received yet.
uint64 RAW_MODE_UNAVAILABLE=0

uint8 state
uint64 raw_robot_mode
bool feedback_fresh
