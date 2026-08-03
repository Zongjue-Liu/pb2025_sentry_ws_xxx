# Posture fields modeled after referee protocol command 0x020D.
uint8 ATTACK = 1
uint8 DEFENSE = 2
uint8 MOBILE = 3
uint8 ENHANCED_ATTACK = 4
uint8 ENHANCED_DEFENSE = 5
uint8 ENHANCED_MOBILE = 6

# Base posture ID: ATTACK, DEFENSE, or MOBILE.
uint8 posture_id
bool is_weakened
bool is_powered

# Remaining time before each base posture becomes weakened, in seconds.
uint16 attack_remaining_time
uint16 defense_remaining_time
uint16 mobile_remaining_time

# Remaining enhanced-posture allowance, in seconds.
float32 enhanced_attack_remaining_time
float32 enhanced_defense_remaining_time
float32 enhanced_mobile_remaining_time

# Simulation-only observability field for the five-second switch cooldown.
float32 switch_cooldown_remaining
