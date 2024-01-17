from . parameterList import *

class ParameterValues:
    def __init__(self):
        self.parameterValues = {
            PARAM_CONFIG_OK : None,
            PARAM_USE_EEPROM : None,
            PARAM_E_STOP_ON_MOV_ERR : None,
            PARAM_MOV_NR_RETRY : None,
            MOVEMENT_TIMEOUT_X : None,
            MOVEMENT_TIMEOUT_Y : None,
            MOVEMENT_TIMEOUT_Z : None,
            MOVEMENT_KEEP_ACTIVE_X : None,
            MOVEMENT_KEEP_ACTIVE_Y : None,
            MOVEMENT_KEEP_ACTIVE_Z : None,
            MOVEMENT_HOME_AT_BOOT_X : None,
            MOVEMENT_HOME_AT_BOOT_Y : None,
            MOVEMENT_HOME_AT_BOOT_Z : None,
            MOVEMENT_INVERT_ENDPOINTS_X : None,
            MOVEMENT_INVERT_ENDPOINTS_Y : None,
            MOVEMENT_INVERT_ENDPOINTS_Z : None,
            MOVEMENT_ENABLE_ENDPOINTS_X : None,
            MOVEMENT_ENABLE_ENDPOINTS_Y : None,
            MOVEMENT_ENABLE_ENDPOINTS_Z : None,
            MOVEMENT_INVERT_MOTOR_X : None,
            MOVEMENT_INVERT_MOTOR_Y : None,
            MOVEMENT_INVERT_MOTOR_Z : None,
            MOVEMENT_SECONDARY_MOTOR_X : None,
            MOVEMENT_SECONDARY_MOTOR_INVERT_X : None,
            MOVEMENT_STEPS_ACC_DEC_X : None,
            MOVEMENT_STEPS_ACC_DEC_Y : None,
            MOVEMENT_STEPS_ACC_DEC_Z : None,
            MOVEMENT_STEPS_ACC_DEC_Z2 : None,
            MOVEMENT_STOP_AT_HOME_X : None,
            MOVEMENT_STOP_AT_HOME_Y : None,
            MOVEMENT_STOP_AT_HOME_Z : None,
            MOVEMENT_HOME_UP_X : None,
            MOVEMENT_HOME_UP_Y : None,
            MOVEMENT_HOME_UP_Z : None,
            MOVEMENT_STEP_PER_MM_X : None,
            MOVEMENT_STEP_PER_MM_Y : None,
            MOVEMENT_STEP_PER_MM_Z : None,
            MOVEMENT_MIN_SPD_X : None,
            MOVEMENT_MIN_SPD_Y : None,
            MOVEMENT_MIN_SPD_Z : None,
            MOVEMENT_MIN_SPD_Z2 : None,
            MOVEMENT_HOME_SPD_X : None,
            MOVEMENT_HOME_SPD_Y : None,
            MOVEMENT_HOME_SPD_Z : None,
            MOVEMENT_MAX_SPD_X : None,
            MOVEMENT_MAX_SPD_Y : None,
            MOVEMENT_MAX_SPD_Z : None,
            MOVEMENT_MAX_SPD_Z2 : None,
            MOVEMENT_INVERT_2_ENDPOINTS_X : None,
            MOVEMENT_INVERT_2_ENDPOINTS_Y : None,
            MOVEMENT_INVERT_2_ENDPOINTS_Z : None,
            MOVEMENT_MOTOR_CURRENT_X : None,
            MOVEMENT_MOTOR_CURRENT_Y : None,
            MOVEMENT_MOTOR_CURRENT_Z : None,
            MOVEMENT_STALL_SENSITIVITY_X : None,
            MOVEMENT_STALL_SENSITIVITY_Y : None,
            MOVEMENT_STALL_SENSITIVITY_Z : None,
            MOVEMENT_MICROSTEPS_X : None,
            MOVEMENT_MICROSTEPS_Y : None,
            MOVEMENT_MICROSTEPS_Z : None,
            ENCODER_ENABLED_X : None,
            ENCODER_ENABLED_Y : None,
            ENCODER_ENABLED_Z : None,
            ENCODER_TYPE_X : None,
            ENCODER_TYPE_Y : None,
            ENCODER_TYPE_Z : None,
            ENCODER_MISSED_STEPS_MAX_X : None,
            ENCODER_MISSED_STEPS_MAX_Y : None,
            ENCODER_MISSED_STEPS_MAX_Z : None,
            ENCODER_SCALING_X : None,
            ENCODER_SCALING_Y : None,
            ENCODER_SCALING_Z : None,
            ENCODER_MISSED_STEPS_DECAY_X : None,
            ENCODER_MISSED_STEPS_DECAY_Y : None,
            ENCODER_MISSED_STEPS_DECAY_Z : None,
            ENCODER_USE_FOR_POS_X : None,
            ENCODER_USE_FOR_POS_Y : None,
            ENCODER_USE_FOR_POS_Z : None,
            ENCODER_INVERT_X : None,
            ENCODER_INVERT_Y : None,
            ENCODER_INVERT_Z : None,
            MOVEMENT_AXIS_NR_STEPS_X : None,
            MOVEMENT_AXIS_NR_STEPS_Y : None,
            MOVEMENT_AXIS_NR_STEPS_Z : None,
            MOVEMENT_STOP_AT_MAX_X : None,
            MOVEMENT_STOP_AT_MAX_Y : None,
            MOVEMENT_STOP_AT_MAX_Z : None,
            MOVEMENT_CALIBRATION_RETRY_X : None,
            MOVEMENT_CALIBRATION_RETRY_Y : None,
            MOVEMENT_CALIBRATION_RETRY_Z : None,
            MOVEMENT_AXIS_STEALTH_X : None,
            MOVEMENT_AXIS_STEALTH_Y : None,
            MOVEMENT_AXIS_STEALTH_Z : None,
            MOVEMENT_CALIBRATION_DEADZONE_X : None,
            MOVEMENT_CALIBRATION_DEADZONE_Y : None,
            MOVEMENT_CALIBRATION_DEADZONE_Z : None,
            MOVEMENT_CALIBRATION_RETRY_TOTAL_X : None,
            MOVEMENT_CALIBRATION_RETRY_TOTAL_Y : None,
            MOVEMENT_CALIBRATION_RETRY_TOTAL_Z : None,
            PIN_REPORT_1_PIN_NR : None,
            PIN_REPORT_2_PIN_NR : None,
            PIN_GUARD_1_PIN_NR : None,
            PIN_GUARD_1_TIME_OUT : None,
            PIN_GUARD_1_ACTIVE_STATE : None,
            PIN_GUARD_2_PIN_NR : None,
            PIN_GUARD_2_TIME_OUT : None,
            PIN_GUARD_2_ACTIVE_STATE : None,
            PIN_GUARD_3_PIN_NR : None,
            PIN_GUARD_3_TIME_OUT : None,
            PIN_GUARD_3_ACTIVE_STATE : None,
            PIN_GUARD_4_PIN_NR : None,
            PIN_GUARD_4_TIME_OUT : None,
            PIN_GUARD_4_ACTIVE_STATE : None,
            PIN_GUARD_5_PIN_NR : None,
            PIN_GUARD_5_TIME_OUT : None,
            PIN_GUARD_5_ACTIVE_STATE : None
        }

    def set_value(self, param, value):
        self.parameterValues[param] = value
    
    def get_value(self, param):
        return self.parameterValues[param]