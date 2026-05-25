class ParameterList:
    @property
    def PARAM_CONFIG_OK(self):
        return 2
    @property
    def PARAM_USE_EEPROM(self):
        return 3
    @property
    def PARAM_E_STOP_ON_MOV_ERR(self):
        return 4
    @property
    def PARAM_MOV_NR_RETRY(self):
        return 5
    @property
    def MOVEMENT_TIMEOUT_X(self):
        return 11
    @property
    def MOVEMENT_TIMEOUT_Y(self):
        return 12
    @property
    def MOVEMENT_TIMEOUT_Z(self):
        return 13
    @property
    def MOVEMENT_KEEP_ACTIVE_X(self):
        return 15
    @property
    def MOVEMENT_KEEP_ACTIVE_Y(self):
        return 16
    @property
    def MOVEMENT_KEEP_ACTIVE_Z(self):
        return 17
    @property
    def MOVEMENT_HOME_AT_BOOT_X(self):
        return 18
    @property
    def MOVEMENT_HOME_AT_BOOT_Y(self):
        return 19
    @property
    def MOVEMENT_HOME_AT_BOOT_Z(self):
        return 20
    @property
    def MOVEMENT_INVERT_ENDPOINTS_X(self):
        return 21
    @property
    def MOVEMENT_INVERT_ENDPOINTS_Y(self):
        return 22
    @property
    def MOVEMENT_INVERT_ENDPOINTS_Z(self):
        return 23
    @property
    def MOVEMENT_ENABLE_ENDPOINTS_X(self):
        return 25
    @property
    def MOVEMENT_ENABLE_ENDPOINTS_Y(self):
        return 26
    @property
    def MOVEMENT_ENABLE_ENDPOINTS_Z(self):
        return 27
    @property
    def MOVEMENT_INVERT_MOTOR_X(self):
        return 31
    @property
    def MOVEMENT_INVERT_MOTOR_Y(self):
        return 32
    @property
    def MOVEMENT_INVERT_MOTOR_Z(self):
        return 33
    @property
    def MOVEMENT_SECONDARY_MOTOR_X(self):
        return 36
    @property
    def MOVEMENT_SECONDARY_MOTOR_INVERT_X(self):
        return 37
    @property
    def MOVEMENT_STEPS_ACC_DEC_X(self):
        return 41
    @property
    def MOVEMENT_STEPS_ACC_DEC_Y(self):
        return 42
    @property
    def MOVEMENT_STEPS_ACC_DEC_Z(self):
        return 43
    @property
    def MOVEMENT_STEPS_ACC_DEC_Z2(self):
        return 44
    @property
    def MOVEMENT_STOP_AT_HOME_X(self):
        return 45
    @property
    def MOVEMENT_STOP_AT_HOME_Y(self):
        return 46
    @property
    def MOVEMENT_STOP_AT_HOME_Z(self):
        return 47
    @property
    def MOVEMENT_HOME_UP_X(self):
        return 51
    @property
    def MOVEMENT_HOME_UP_Y(self):
        return 52
    @property
    def MOVEMENT_HOME_UP_Z(self):
        return 53
    @property
    def MOVEMENT_STEP_PER_MM_X(self):
        return 55
    @property
    def MOVEMENT_STEP_PER_MM_Y(self):
        return 56
    @property
    def MOVEMENT_STEP_PER_MM_Z(self):
        return 57
    @property
    def MOVEMENT_MIN_SPD_X(self):
        return 61
    @property
    def MOVEMENT_MIN_SPD_Y(self):
        return 62
    @property
    def MOVEMENT_MIN_SPD_Z(self):
        return 63
    @property
    def MOVEMENT_MIN_SPD_Z2(self):
        return 64
    @property
    def MOVEMENT_HOME_SPD_X(self):
        return 65
    @property
    def MOVEMENT_HOME_SPD_Y(self):
        return 66
    @property
    def MOVEMENT_HOME_SPD_Z(self):
        return 67
    @property
    def MOVEMENT_MAX_SPD_X(self):
        return 71
    @property
    def MOVEMENT_MAX_SPD_Y(self):
        return 72
    @property
    def MOVEMENT_MAX_SPD_Z(self):
        return 73
    @property
    def MOVEMENT_MAX_SPD_Z2(self):
        return 74
    @property
    def MOVEMENT_INVERT_2_ENDPOINTS_X(self):
        return 75
    @property
    def MOVEMENT_INVERT_2_ENDPOINTS_Y(self):
        return 76
    @property
    def MOVEMENT_INVERT_2_ENDPOINTS_Z(self):
        return 77
    @property
    def MOVEMENT_MOTOR_CURRENT_X(self):
        return 81
    @property
    def MOVEMENT_MOTOR_CURRENT_Y(self):
        return 82
    @property
    def MOVEMENT_MOTOR_CURRENT_Z(self):
        return 83
    @property
    def MOVEMENT_STALL_SENSITIVITY_X(self):
        return 85
    @property
    def MOVEMENT_STALL_SENSITIVITY_Y(self):
        return 86
    @property
    def MOVEMENT_STALL_SENSITIVITY_Z(self):
        return 87
    @property
    def MOVEMENT_MICROSTEPS_X(self):
        return 91
    @property
    def MOVEMENT_MICROSTEPS_Y(self):
        return 92
    @property
    def MOVEMENT_MICROSTEPS_Z(self):
        return 93
    @property
    def ENCODER_ENABLED_X(self):
        return 101
    @property
    def ENCODER_ENABLED_Y(self):
        return 102
    @property
    def ENCODER_ENABLED_Z(self):
        return 103
    @property
    def ENCODER_TYPE_X(self):
        return 105
    @property
    def ENCODER_TYPE_Y(self):
        return 106
    @property
    def ENCODER_TYPE_Z(self):
        return 107
    @property
    def ENCODER_MISSED_STEPS_MAX_X(self):
        return 111
    @property
    def ENCODER_MISSED_STEPS_MAX_Y(self):
        return 112
    @property
    def ENCODER_MISSED_STEPS_MAX_Z(self):
        return 113
    @property
    def ENCODER_SCALING_X(self):
        return 115
    @property
    def ENCODER_SCALING_Y(self):
        return 116
    @property
    def ENCODER_SCALING_Z(self):
        return 117
    @property
    def ENCODER_MISSED_STEPS_DECAY_X(self):
        return 121
    @property
    def ENCODER_MISSED_STEPS_DECAY_Y(self):
        return 122
    @property
    def ENCODER_MISSED_STEPS_DECAY_Z(self):
        return 123
    @property
    def ENCODER_USE_FOR_POS_X(self):
        return 125
    @property
    def ENCODER_USE_FOR_POS_Y(self):
        return 126
    @property
    def ENCODER_USE_FOR_POS_Z(self):
        return 127
    @property
    def ENCODER_INVERT_X(self):
        return 131
    @property
    def ENCODER_INVERT_Y(self):
        return 132
    @property
    def ENCODER_INVERT_Z(self):
        return 133
    @property
    def MOVEMENT_AXIS_NR_STEPS_X(self):
        return 141
    @property
    def MOVEMENT_AXIS_NR_STEPS_Y(self):
        return 142
    @property
    def MOVEMENT_AXIS_NR_STEPS_Z(self):
        return 143
    @property
    def MOVEMENT_STOP_AT_MAX_X(self):
        return 145
    @property
    def MOVEMENT_STOP_AT_MAX_Y(self):
        return 146
    @property
    def MOVEMENT_STOP_AT_MAX_Z(self):
        return 147
    @property
    def MOVEMENT_CALIBRATION_RETRY_X(self):
        return 161
    @property
    def MOVEMENT_CALIBRATION_RETRY_Y(self):
        return 162
    @property
    def MOVEMENT_CALIBRATION_RETRY_Z(self):
        return 163
    @property
    def MOVEMENT_AXIS_STEALTH_X(self):
        return 165
    @property
    def MOVEMENT_AXIS_STEALTH_Y(self):
        return 166
    @property
    def MOVEMENT_AXIS_STEALTH_Z(self):
        return 167
    @property
    def MOVEMENT_CALIBRATION_DEADZONE_X(self):
        return 171
    @property
    def MOVEMENT_CALIBRATION_DEADZONE_Y(self):
        return 172
    @property
    def MOVEMENT_CALIBRATION_DEADZONE_Z(self):
        return 173
    @property
    def MOVEMENT_CALIBRATION_RETRY_TOTAL_X(self):
        return 175
    @property
    def MOVEMENT_CALIBRATION_RETRY_TOTAL_Y(self):
        return 176
    @property
    def MOVEMENT_CALIBRATION_RETRY_TOTAL_Z(self):
        return 177
    @property
    def PIN_REPORT_1_PIN_NR(self):
        return 198
    @property
    def PIN_REPORT_2_PIN_NR(self):
        return 199
    @property
    def PIN_GUARD_1_PIN_NR(self):
        return 201
    @property
    def PIN_GUARD_1_TIME_OUT(self):
        return 202
    @property
    def PIN_GUARD_1_ACTIVE_STATE(self):
        return 203
    @property
    def PIN_GUARD_2_PIN_NR(self):
        return 205
    @property
    def PIN_GUARD_2_TIME_OUT(self):
        return 206
    @property
    def PIN_GUARD_2_ACTIVE_STATE(self):
        return 207
    @property
    def PIN_GUARD_3_PIN_NR(self):
        return 211
    @property
    def PIN_GUARD_3_TIME_OUT(self):
        return 212
    @property
    def PIN_GUARD_3_ACTIVE_STATE(self):
        return 213
    @property
    def PIN_GUARD_4_PIN_NR(self):
        return 215
    @property
    def PIN_GUARD_4_TIME_OUT(self):
        return 216
    @property
    def PIN_GUARD_4_ACTIVE_STATE(self):
        return 217
    @property
    def PIN_GUARD_5_PIN_NR(self):
        return 221
    @property
    def PIN_GUARD_5_TIME_OUT(self):
        return 222
    @property
    def PIN_GUARD_5_ACTIVE_STATE(self):
        return 223
