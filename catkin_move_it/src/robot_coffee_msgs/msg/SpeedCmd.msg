# Speed cmd
# Options defined below
float32 speed_cmd
uint8 speed_cmd_type

# Enable
bool enable

# Clear driver overrides
bool clear

# Ignore driver overrides
bool ignore

# Watchdog counter (optional)
uint8 count

uint8 CMD_NONE=0
uint8 CMD_KPH=1     # KPH, range 0 to 10kph
uint8 CMD_PERCENT=2 # Percent of maximum throttle, range 0 to 1

