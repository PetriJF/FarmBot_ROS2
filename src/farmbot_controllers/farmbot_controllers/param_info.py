"""Arduino parameters numbers."""


class ParameterList:
    """Contains the identifiers for the Arduino parameters."""

    @property
    def PARAM_CONFIG_OK(self):
        """
        Id of PARAM_CONFIG_OK.

        Parameter unit: 0/1
        """
        return 2

    @property
    def PARAM_USE_EEPROM(self):
        """
        Id of PARAM_USE_EEPROM.

        Parameter unit: 0/1
        """
        return 3

    @property
    def PARAM_E_STOP_ON_MOV_ERR(self):
        """
        Id of PARAM_E_STOP_ON_MOV_ERR.

        Parameter unit: 0/1
        """
        return 4

    @property
    def PARAM_MOV_NR_RETRY(self):
        """
        Id of PARAM_MOV_NR_RETRY.

        Parameter unit: integer
        """
        return 5

    @property
    def MOVEMENT_TIMEOUT_X(self):
        """
        Id of MOVEMENT_TIMEOUT_X.

        Parameter unit: seconds
        """
        return 11

    @property
    def MOVEMENT_TIMEOUT_Y(self):
        """
        Id of MOVEMENT_TIMEOUT_Y.

        Parameter unit: seconds
        """
        return 12

    @property
    def MOVEMENT_TIMEOUT_Z(self):
        """
        Id of MOVEMENT_TIMEOUT_Z.

        Parameter unit: seconds
        """
        return 13

    @property
    def MOVEMENT_KEEP_ACTIVE_X(self):
        """
        Id of MOVEMENT_KEEP_ACTIVE_X.

        Parameter unit: 0/1
        """
        return 15

    @property
    def MOVEMENT_KEEP_ACTIVE_Y(self):
        """
        Id of MOVEMENT_KEEP_ACTIVE_Y.

        Parameter unit: 0/1
        """
        return 16

    @property
    def MOVEMENT_KEEP_ACTIVE_Z(self):
        """
        Id of MOVEMENT_KEEP_ACTIVE_Z.

        Parameter unit: 0/1
        """
        return 17

    @property
    def MOVEMENT_HOME_AT_BOOT_X(self):
        """
        Id of MOVEMENT_HOME_AT_BOOT_X.

        Parameter unit: 0/1
        """
        return 18

    @property
    def MOVEMENT_HOME_AT_BOOT_Y(self):
        """
        Id of MOVEMENT_HOME_AT_BOOT_Y.

        Parameter unit: 0/1
        """
        return 19

    @property
    def MOVEMENT_HOME_AT_BOOT_Z(self):
        """
        Id of MOVEMENT_HOME_AT_BOOT_Z.

        Parameter unit: 0/1
        """
        return 20

    @property
    def MOVEMENT_INVERT_ENDPOINTS_X(self):
        """
        Id of MOVEMENT_INVERT_ENDPOINTS_X.

        Parameter unit: 0/1
        Notes: switch ends
        """
        return 21

    @property
    def MOVEMENT_INVERT_ENDPOINTS_Y(self):
        """
        Id of MOVEMENT_INVERT_ENDPOINTS_Y.

        Parameter unit: 0/1
        Notes: switch ends
        """
        return 22

    @property
    def MOVEMENT_INVERT_ENDPOINTS_Z(self):
        """
        Id of MOVEMENT_INVERT_ENDPOINTS_Z.

        Parameter unit: 0/1
        Notes: switch ends
        """
        return 23

    @property
    def MOVEMENT_ENABLE_ENDPOINTS_X(self):
        """
        Id of MOVEMENT_ENABLE_ENDPOINTS_X.

        Parameter unit: 0/1
        """
        return 25

    @property
    def MOVEMENT_ENABLE_ENDPOINTS_Y(self):
        """
        Id of MOVEMENT_ENABLE_ENDPOINTS_Y.

        Parameter unit: 0/1
        """
        return 26

    @property
    def MOVEMENT_ENABLE_ENDPOINTS_Z(self):
        """
        Id of MOVEMENT_ENABLE_ENDPOINTS_Z.

        Parameter unit: 0/1
        """
        return 27

    @property
    def MOVEMENT_INVERT_MOTOR_X(self):
        """
        Id of MOVEMENT_INVERT_MOTOR_X.

        Parameter unit: 0/1
        """
        return 31

    @property
    def MOVEMENT_INVERT_MOTOR_Y(self):
        """
        Id of MOVEMENT_INVERT_MOTOR_Y.

        Parameter unit: 0/1
        """
        return 32

    @property
    def MOVEMENT_INVERT_MOTOR_Z(self):
        """
        Id of MOVEMENT_INVERT_MOTOR_Z.

        Parameter unit: 0/1
        """
        return 33

    @property
    def MOVEMENT_SECONDARY_MOTOR_X(self):
        """
        Id of MOVEMENT_SECONDARY_MOTOR_X.

        Parameter unit: 0/1
        """
        return 36

    @property
    def MOVEMENT_SECONDARY_MOTOR_INVERT_X(self):
        """
        Id of MOVEMENT_SECONDARY_MOTOR_INVERT_X.

        Parameter unit: 0/1
        """
        return 37

    @property
    def MOVEMENT_STEPS_ACC_DEC_X(self):
        """
        Id of MOVEMENT_STEPS_ACC_DEC_X.

        Parameter unit: steps
        """
        return 41

    @property
    def MOVEMENT_STEPS_ACC_DEC_Y(self):
        """
        Id of MOVEMENT_STEPS_ACC_DEC_Y.

        Parameter unit: steps
        """
        return 42

    @property
    def MOVEMENT_STEPS_ACC_DEC_Z(self):
        """
        Id of MOVEMENT_STEPS_ACC_DEC_Z.

        Parameter unit: steps
        Notes: (away from home)
        """
        return 43

    @property
    def MOVEMENT_STEPS_ACC_DEC_Z2(self):
        """
        Id of MOVEMENT_STEPS_ACC_DEC_Z2.

        Parameter unit: steps
        Notes: (toward home)
        """
        return 44

    @property
    def MOVEMENT_STOP_AT_HOME_X(self):
        """
        Id of MOVEMENT_STOP_AT_HOME_X.

        Parameter unit: 0/1
        """
        return 45

    @property
    def MOVEMENT_STOP_AT_HOME_Y(self):
        """
        Id of MOVEMENT_STOP_AT_HOME_Y.

        Parameter unit: 0/1
        """
        return 46

    @property
    def MOVEMENT_STOP_AT_HOME_Z(self):
        """
        Id of MOVEMENT_STOP_AT_HOME_Z.

        Parameter unit: 0/1
        """
        return 47

    @property
    def MOVEMENT_HOME_UP_X(self):
        """
        Id of MOVEMENT_HOME_UP_X.

        Parameter unit: 0/1
        """
        return 51

    @property
    def MOVEMENT_HOME_UP_Y(self):
        """
        Id of MOVEMENT_HOME_UP_Y.

        Parameter unit: 0/1
        """
        return 52

    @property
    def MOVEMENT_HOME_UP_Z(self):
        """
        Id of MOVEMENT_HOME_UP_Z.

        Parameter unit: 0/1
        """
        return 53

    @property
    def MOVEMENT_STEP_PER_MM_X(self):
        """
        Id of MOVEMENT_STEP_PER_MM_X.

        Parameter unit: steps
        """
        return 55

    @property
    def MOVEMENT_STEP_PER_MM_Y(self):
        """
        Id of MOVEMENT_STEP_PER_MM_Y.

        Parameter unit: steps
        """
        return 56

    @property
    def MOVEMENT_STEP_PER_MM_Z(self):
        """
        Id of MOVEMENT_STEP_PER_MM_Z.

        Parameter unit: steps
        """
        return 57

    @property
    def MOVEMENT_MIN_SPD_X(self):
        """
        Id of MOVEMENT_MIN_SPD_X.

        Parameter unit: steps/s
        """
        return 61

    @property
    def MOVEMENT_MIN_SPD_Y(self):
        """
        Id of MOVEMENT_MIN_SPD_Y.

        Parameter unit: steps/s
        """
        return 62

    @property
    def MOVEMENT_MIN_SPD_Z(self):
        """
        Id of MOVEMENT_MIN_SPD_Z.

        Parameter unit: steps/s
        Notes: (away from home)
        """
        return 63

    @property
    def MOVEMENT_MIN_SPD_Z2(self):
        """
        Id of MOVEMENT_MIN_SPD_Z2.

        Parameter unit: steps/s
        Notes: (toward home)
        """
        return 64

    @property
    def MOVEMENT_HOME_SPD_X(self):
        """
        Id of MOVEMENT_HOME_SPD_X.

        Parameter unit: steps/s
        """
        return 65

    @property
    def MOVEMENT_HOME_SPD_Y(self):
        """
        Id of MOVEMENT_HOME_SPD_Y.

        Parameter unit: steps/s
        """
        return 66

    @property
    def MOVEMENT_HOME_SPD_Z(self):
        """
        Id of MOVEMENT_HOME_SPD_Z.

        Parameter unit: steps/s
        """
        return 67

    @property
    def MOVEMENT_MAX_SPD_X(self):
        """
        Id of MOVEMENT_MAX_SPD_X.

        Parameter unit: steps/s
        """
        return 71

    @property
    def MOVEMENT_MAX_SPD_Y(self):
        """
        Id of MOVEMENT_MAX_SPD_Y.

        Parameter unit: steps/s
        """
        return 72

    @property
    def MOVEMENT_MAX_SPD_Z(self):
        """
        Id of MOVEMENT_MAX_SPD_Z.

        Parameter unit: steps/s
        Notes: (away from home)
        """
        return 73

    @property
    def MOVEMENT_MAX_SPD_Z2(self):
        """
        Id of MOVEMENT_MAX_SPD_Z2.

        Parameter unit: steps/s
        Notes: (toward home)
        """
        return 74

    @property
    def MOVEMENT_INVERT_2_ENDPOINTS_X(self):
        """
        Id of MOVEMENT_INVERT_2_ENDPOINTS_X.

        Parameter unit: 0/1
        Notes: switch NO and NC
        """
        return 75

    @property
    def MOVEMENT_INVERT_2_ENDPOINTS_Y(self):
        """
        Id of MOVEMENT_INVERT_2_ENDPOINTS_Y.

        Parameter unit: 0/1
        Notes: switch NO and NC
        """
        return 76

    @property
    def MOVEMENT_INVERT_2_ENDPOINTS_Z(self):
        """
        Id of MOVEMENT_INVERT_2_ENDPOINTS_Z.

        Parameter unit: 0/1
        Notes: switch NO and NC
        """
        return 77

    @property
    def MOVEMENT_MOTOR_CURRENT_X(self):
        """
        Id of MOVEMENT_MOTOR_CURRENT_X.

        Parameter unit: mA
        Notes: TMC2130 only
        """
        return 81

    @property
    def MOVEMENT_MOTOR_CURRENT_Y(self):
        """
        Id of MOVEMENT_MOTOR_CURRENT_Y.

        Parameter unit: mA
        Notes: TMC2130 only
        """
        return 82

    @property
    def MOVEMENT_MOTOR_CURRENT_Z(self):
        """
        Id of MOVEMENT_MOTOR_CURRENT_Z.

        Parameter unit: mA
        Notes: TMC2130 only
        """
        return 83

    @property
    def MOVEMENT_STALL_SENSITIVITY_X(self):
        """
        Id of MOVEMENT_STALL_SENSITIVITY_X.

        Parameter unit: integer
        Notes: -63 (high) to +63 (low), Express only
        """
        return 85

    @property
    def MOVEMENT_STALL_SENSITIVITY_Y(self):
        """
        Id of MOVEMENT_STALL_SENSITIVITY_Y.

        Parameter unit: integer
        Notes: -63 (high) to +63 (low), Express only
        """
        return 86

    @property
    def MOVEMENT_STALL_SENSITIVITY_Z(self):
        """
        Id of MOVEMENT_STALL_SENSITIVITY_Z.

        Parameter unit: integer
        Notes: -63 (high) to +63 (low), Express only
        """
        return 87

    @property
    def MOVEMENT_MICROSTEPS_X(self):
        """
        Id of MOVEMENT_MICROSTEPS_X.

        Parameter unit: integer
        Notes: TMC2130 only
        """
        return 91

    @property
    def MOVEMENT_MICROSTEPS_Y(self):
        """
        Id of MOVEMENT_MICROSTEPS_Y.

        Parameter unit: integer
        Notes: TMC2130 only
        """
        return 92

    @property
    def MOVEMENT_MICROSTEPS_Z(self):
        """
        Id of MOVEMENT_MICROSTEPS_Z.

        Parameter unit: integer
        Notes: TMC2130 only
        """
        return 93

    @property
    def ENCODER_ENABLED_X(self):
        """
        Id of ENCODER_ENABLED_X.

        Parameter unit: 0/1
        Notes: enables stall detection on Express
        """
        return 101

    @property
    def ENCODER_ENABLED_Y(self):
        """
        Id of ENCODER_ENABLED_Y.

        Parameter unit: 0/1
        Notes: enables stall detection on Express
        """
        return 102

    @property
    def ENCODER_ENABLED_Z(self):
        """
        Id of ENCODER_ENABLED_Z.

        Parameter unit: 0/1
        Notes: enables stall detection on Express
        """
        return 103

    @property
    def ENCODER_TYPE_X(self):
        """
        Id of ENCODER_TYPE_X.

        Parameter unit: 0
        Notes: differential channels disabled
        """
        return 105

    @property
    def ENCODER_TYPE_Y(self):
        """
        Id of ENCODER_TYPE_Y.

        Parameter unit: 0
        Notes: differential channels disabled
        """
        return 106

    @property
    def ENCODER_TYPE_Z(self):
        """
        Id of ENCODER_TYPE_Z.

        Parameter unit: 0
        Notes: differential channels disabled
        """
        return 107

    @property
    def ENCODER_MISSED_STEPS_MAX_X(self):
        """
        Id of ENCODER_MISSED_STEPS_MAX_X.

        Parameter unit: steps
        """
        return 111

    @property
    def ENCODER_MISSED_STEPS_MAX_Y(self):
        """
        Id of ENCODER_MISSED_STEPS_MAX_Y.

        Parameter unit: steps
        """
        return 112

    @property
    def ENCODER_MISSED_STEPS_MAX_Z(self):
        """
        Id of ENCODER_MISSED_STEPS_MAX_Z.

        Parameter unit: steps
        """
        return 113

    @property
    def ENCODER_SCALING_X(self):
        """
        Id of ENCODER_SCALING_X.

        Parameter unit: integer
        Notes: 10000*motor/encoder (except Express)
        """
        return 115

    @property
    def ENCODER_SCALING_Y(self):
        """
        Id of ENCODER_SCALING_Y.

        Parameter unit: integer
        Notes: 10000*motor/encoder (except Express)
        """
        return 116

    @property
    def ENCODER_SCALING_Z(self):
        """
        Id of ENCODER_SCALING_Z.

        Parameter unit: integer
        Notes: 10000*motor/encoder (except Express)
        """
        return 117

    @property
    def ENCODER_MISSED_STEPS_DECAY_X(self):
        """
        Id of ENCODER_MISSED_STEPS_DECAY_X.

        Parameter unit: steps
        Notes: 1-99
        """
        return 121

    @property
    def ENCODER_MISSED_STEPS_DECAY_Y(self):
        """
        Id of ENCODER_MISSED_STEPS_DECAY_Y.

        Parameter unit: steps
        Notes: 1-99
        """
        return 122

    @property
    def ENCODER_MISSED_STEPS_DECAY_Z(self):
        """
        Id of ENCODER_MISSED_STEPS_DECAY_Z.

        Parameter unit: steps
        Notes: 1-99
        """
        return 123

    @property
    def ENCODER_USE_FOR_POS_X(self):
        """
        Id of ENCODER_USE_FOR_POS_X.

        Parameter unit: 0/1
        Notes: except Express
        """
        return 125

    @property
    def ENCODER_USE_FOR_POS_Y(self):
        """
        Id of ENCODER_USE_FOR_POS_Y.

        Parameter unit: 0/1
        Notes: except Express
        """
        return 126

    @property
    def ENCODER_USE_FOR_POS_Z(self):
        """
        Id of ENCODER_USE_FOR_POS_Z.

        Parameter unit: 0/1
        Notes: except Express
        """
        return 127

    @property
    def ENCODER_INVERT_X(self):
        """
        Id of ENCODER_INVERT_X.

        Parameter unit: 0/1
        Notes: except Express
        """
        return 131

    @property
    def ENCODER_INVERT_Y(self):
        """
        Id of ENCODER_INVERT_Y.

        Parameter unit: 0/1
        Notes: except Express
        """
        return 132

    @property
    def ENCODER_INVERT_Z(self):
        """
        Id of ENCODER_INVERT_Z.

        Parameter unit: 0/1
        Notes: except Express
        """
        return 133

    @property
    def MOVEMENT_AXIS_NR_STEPS_X(self):
        """
        Id of MOVEMENT_AXIS_NR_STEPS_X.

        Parameter unit: steps
        Notes: 0 = limit disabled
        """
        return 141

    @property
    def MOVEMENT_AXIS_NR_STEPS_Y(self):
        """
        Id of MOVEMENT_AXIS_NR_STEPS_Y.

        Parameter unit: steps
        Notes: 0 = limit disabled
        """
        return 142

    @property
    def MOVEMENT_AXIS_NR_STEPS_Z(self):
        """
        Id of MOVEMENT_AXIS_NR_STEPS_Z.

        Parameter unit: steps
        Notes: 0 = limit disabled
        """
        return 143

    @property
    def MOVEMENT_STOP_AT_MAX_X(self):
        """
        Id of MOVEMENT_STOP_AT_MAX_X.

        Parameter unit: 0/1
        """
        return 145

    @property
    def MOVEMENT_STOP_AT_MAX_Y(self):
        """
        Id of MOVEMENT_STOP_AT_MAX_Y.

        Parameter unit: 0/1
        """
        return 146

    @property
    def MOVEMENT_STOP_AT_MAX_Z(self):
        """
        Id of MOVEMENT_STOP_AT_MAX_Z.

        Parameter unit: 0/1
        """
        return 147

    @property
    def MOVEMENT_CALIBRATION_RETRY_X(self):
        """
        Id of MOVEMENT_CALIBRATION_RETRY_X.

        Parameter unit: integer
        """
        return 161

    @property
    def MOVEMENT_CALIBRATION_RETRY_Y(self):
        """
        Id of MOVEMENT_CALIBRATION_RETRY_Y.

        Parameter unit: integer
        """
        return 162

    @property
    def MOVEMENT_CALIBRATION_RETRY_Z(self):
        """
        Id of MOVEMENT_CALIBRATION_RETRY_Z.

        Parameter unit: integer
        """
        return 163

    @property
    def MOVEMENT_AXIS_STEALTH_X(self):
        """
        Id of MOVEMENT_AXIS_STEALTH_X.

        Parameter unit: 0/1
        """
        return 165

    @property
    def MOVEMENT_AXIS_STEALTH_Y(self):
        """
        Id of MOVEMENT_AXIS_STEALTH_Y.

        Parameter unit: 0/1
        """
        return 166

    @property
    def MOVEMENT_AXIS_STEALTH_Z(self):
        """
        Id of MOVEMENT_AXIS_STEALTH_Z.

        Parameter unit: 0/1
        """
        return 167

    @property
    def MOVEMENT_CALIBRATION_DEADZONE_X(self):
        """
        Id of MOVEMENT_CALIBRATION_DEADZONE_X.

        Parameter unit: integer
        """
        return 171

    @property
    def MOVEMENT_CALIBRATION_DEADZONE_Y(self):
        """
        Id of MOVEMENT_CALIBRATION_DEADZONE_Y.

        Parameter unit: integer
        """
        return 172

    @property
    def MOVEMENT_CALIBRATION_DEADZONE_Z(self):
        """
        Id of MOVEMENT_CALIBRATION_DEADZONE_Z.

        Parameter unit: integer
        """
        return 173

    @property
    def MOVEMENT_CALIBRATION_RETRY_TOTAL_X(self):
        """
        Id of MOVEMENT_CALIBRATION_RETRY_TOTAL_X.

        Parameter unit: integer
        """
        return 175

    @property
    def MOVEMENT_CALIBRATION_RETRY_TOTAL_Y(self):
        """
        Id of MOVEMENT_CALIBRATION_RETRY_TOTAL_Y.

        Parameter unit: integer
        """
        return 176

    @property
    def MOVEMENT_CALIBRATION_RETRY_TOTAL_Z(self):
        """
        Id of MOVEMENT_CALIBRATION_RETRY_TOTAL_Z.

        Parameter unit: integer
        """
        return 177

    @property
    def PIN_REPORT_1_PIN_NR(self):
        """
        Id of PIN_REPORT_1_PIN_NR.

        Parameter unit: integer
        Notes: reports every 500ms, 0 = disabled
        """
        return 198

    @property
    def PIN_REPORT_2_PIN_NR(self):
        """
        Id of PIN_REPORT_2_PIN_NR.

        Parameter unit: integer
        Notes: reports every 500ms, 0 = disabled
        """
        return 199

    @property
    def PIN_GUARD_1_PIN_NR(self):
        """
        Id of PIN_GUARD_1_PIN_NR.

        Parameter unit: integer
        """
        return 201

    @property
    def PIN_GUARD_1_TIME_OUT(self):
        """
        Id of PIN_GUARD_1_TIME_OUT.

        Parameter unit: seconds
        """
        return 202

    @property
    def PIN_GUARD_1_ACTIVE_STATE(self):
        """
        Id of PIN_GUARD_1_ACTIVE_STATE.

        Parameter unit: 0/1
        """
        return 203

    @property
    def PIN_GUARD_2_PIN_NR(self):
        """
        Id of PIN_GUARD_2_PIN_NR.

        Parameter unit: integer
        """
        return 205

    @property
    def PIN_GUARD_2_TIME_OUT(self):
        """
        Id of PIN_GUARD_2_TIME_OUT.

        Parameter unit: seconds
        """
        return 206

    @property
    def PIN_GUARD_2_ACTIVE_STATE(self):
        """
        Id of PIN_GUARD_2_ACTIVE_STATE.

        Parameter unit: 0/1
        """
        return 207

    @property
    def PIN_GUARD_3_PIN_NR(self):
        """
        Id of PIN_GUARD_3_PIN_NR.

        Parameter unit: integer
        """
        return 211

    @property
    def PIN_GUARD_3_TIME_OUT(self):
        """
        Id of PIN_GUARD_3_TIME_OUT.

        Parameter unit: seconds
        """
        return 212

    @property
    def PIN_GUARD_3_ACTIVE_STATE(self):
        """
        Id of PIN_GUARD_3_ACTIVE_STATE.

        Parameter unit: 0/1
        """
        return 213

    @property
    def PIN_GUARD_4_PIN_NR(self):
        """
        Id of PIN_GUARD_4_PIN_NR.

        Parameter unit: integer
        """
        return 215

    @property
    def PIN_GUARD_4_TIME_OUT(self):
        """
        Id of PIN_GUARD_4_TIME_OUT.

        Parameter unit: seconds
        """
        return 216

    @property
    def PIN_GUARD_4_ACTIVE_STATE(self):
        """
        Id of PIN_GUARD_4_ACTIVE_STATE.

        Parameter unit: 0/1
        """
        return 217

    @property
    def PIN_GUARD_5_PIN_NR(self):
        """
        Id of PIN_GUARD_5_PIN_NR.

        Parameter unit: integer
        """
        return 221

    @property
    def PIN_GUARD_5_TIME_OUT(self):
        """
        Id of PIN_GUARD_5_TIME_OUT.

        Parameter unit: seconds
        """
        return 222

    @property
    def PIN_GUARD_5_ACTIVE_STATE(self):
        """
        Id of PIN_GUARD_5_ACTIVE_STATE.

        Parameter unit: 0/1
        """
        return 223
