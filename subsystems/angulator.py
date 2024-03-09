import math

import wpilib
import commands2
import commands2.cmd
from commands2 import *
from commands2.cmd import * 
import ntcore 
import constants
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut, VelocityVoltage, MotionMagicVoltage
from phoenix6.signals.spn_enums import *
from phoenix6.status_code import StatusCode
from talonfxextended import TalonFXExtended
from wpilib import SmartDashboard
from wpilib.sysid import SysIdRoutineLog
from wpiutil.log import StringLogEntry
from wpilib import DataLogManager
from commands2 import sysid
from phoenix6.configs.cancoder_configs import CANcoderConfiguration 

from phoenix6.hardware.cancoder import CANcoder



class AngulatorSubsystem(Subsystem):
    def log_state(self, state):
        if not self.__loggerState:
            self.__loggerState = StringLogEntry(DataLogManager.getLog(),
                                                "sysid-test-state-angulator")
        self.__loggerState.append(sysid.SysIdRoutine.stateEnumToString(state))    
                  

    def move_voltage(self, v):
        self.angulatorMotor.set_control(VoltageOut(v, enable_foc=False)) 
       
        
    def move_log(self, log: SysIdRoutineLog):
        log.motor("angulator") \
            .voltage(self.angulatorMotor.get_motor_voltage().value) \
            .velocity(self.angulatorMotor.get_velocity().value) \
            .position(self.angulatorMotor.get_absolute_position().value)


    def sysid_quasistatic_cmd(self, direction):
        return self.__sysid.quasistatic(direction)
        
    
    def sysid_dynamic_cmd(self, direction):
        return self.__sysid.dynamic(direction)


    def __init__(self):
        super().__init__()
        self.__loggerState = None
        self.setpoint = 0
        
        self.angulatorMotor = TalonFXExtended(constants.kAngulatorMotorCANID, "2481")
        
        self.__sysid_config = sysid.SysIdRoutine.Config(recordState=self.log_state)
        self.__sysid_mechanism = sysid.SysIdRoutine.Mechanism(self.move_voltage, self.move_log, self, "angulator")
        self.__sysid = sysid.SysIdRoutine(self.__sysid_config, self.__sysid_mechanism)

        self.encoder_offset = wpilib.Preferences.getDouble("ANGULATOR_OFFSET", 0.0)

        self.angulatorMotorConfig = TalonFXConfiguration()
        self.angulatorMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.angulatorMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.angulatorMotorConfig.slot0.k_p = constants.kAngulatorP
        self.angulatorMotorConfig.slot0.k_i = constants.kAngulatorI
        self.angulatorMotorConfig.slot0.k_d = constants.kAngulatorD
        self.angulatorMotorConfig.slot0.k_v = constants.kAngulatorV
        self.angulatorMotorConfig.slot0.k_a = constants.kAngulatorA
        self.angulatorMotorConfig.slot0.k_s = constants.kAngulatorS
        self.angulatorMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kAngulatorCruiseVelocity
        self.angulatorMotorConfig.motion_magic.motion_magic_acceleration = constants.kAngulatorAcceleration
        self.angulatorMotorConfig.motion_magic.motion_magic_jerk = constants.kAngulatorJerk
        
        
        self.angulatorMotorConfig.software_limit_switch.forward_soft_limit_threshold = constants.kAngulatorForwardSoftLimitRot + self.encoder_offset
        self.angulatorMotorConfig.software_limit_switch.reverse_soft_limit_threshold = constants.kAngulatorReverseSoftLimitRot + self.encoder_offset
        self.angulatorMotorConfig.software_limit_switch.forward_soft_limit_enable = True
        self.angulatorMotorConfig.software_limit_switch.reverse_soft_limit_enable =  True
        
        self.canCoderConfig = CANcoderConfiguration()
        self.angulatorEncoder = CANcoder(constants.kAngulatorEncoderCANID, "2481")
        self.canCoderConfig.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        self.canCoderConfig.magnet_sensor.sensor_direction = SensorDirectionValue.CLOCKWISE_POSITIVE
        self.canCoderConfig.magnet_sensor.magnet_offset = 0
        
        self.apply_encoder_config_with_retries(self.canCoderConfig)
        
        self.angulatorMotorConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        self.angulatorMotorConfig.feedback.feedback_remote_sensor_id = constants.kAngulatorEncoderCANID 
        self.angulatorMotorConfig.feedback.sensor_to_mechanism_ratio = 1.0
        self.angulatorMotorConfig.feedback.rotor_to_sensor_ratio = 259.9 # 353.68
        self.angulatorMotor.configurator.apply(self.angulatorMotorConfig)

        # self.angulatorEncoder.set_position(0)

          
    def get_error(self):
        return self.setpoint - self.angulatorEncoder.get_absolute_position().value


    def set_angulator_position(self, position):
        position += self.encoder_offset
        self.setpoint = position
        self.angulatorMotor.set_control(MotionMagicVoltage(position=position))
    

    def angulator_set_pos_cmd(self, angulator_position):
        return FunctionalCommand(
            lambda: self.set_angulator_position(angulator_position),
            lambda: None,
            lambda interrupted: None,
            lambda: math.fabs(self.get_error()) < 0.05,
        ).withTimeout(0.5)
    

    def angulator_inc_pos_target(self, delta):
        self.setpoint = self.setpoint + delta
        self.angulatorMotor.set_control(MotionMagicVoltage(position=self.setpoint))


    def angulator_up_cmd(self):
        return FunctionalCommand(
            lambda: self.angulator_inc_pos_target(.001),
            lambda: None,
            lambda interrupted: None,
            lambda: math.fabs(self.get_error()) < 0.0005,
            self
        ).withTimeout(0.5)

    
    def angulator_down_cmd(self):
        return FunctionalCommand(
            lambda: self.angulator_inc_pos_target(-.001),
            lambda: None,
            lambda interrupted: None,
            lambda: math.fabs(self.get_error()) < 0.0005,
            self
        ).withTimeout(0.5)

   
    def angulator_amp_handoff_cmd(self):
        return self.angulator_set_pos_cmd(0.13)


    def set_pos_from_range(self, range_cb):
        HEIGHT_OF_TARGET = 1.98
        angulator_angle = math.degrees(math.atan(HEIGHT_OF_TARGET/range_cb())) - 22
         # TODO: Put this in constant
        angulator_rotation = angulator_angle / 360.0
        SmartDashboard.putNumber("Angulator Angle for Speaker", angulator_angle)
        SmartDashboard.putNumber("Angulator Rotation for Speaker", angulator_rotation)
        self.angulatorMotor.set_control(MotionMagicVoltage(position=angulator_rotation+self.encoder_offset))


    def angulator_set_pos_from_range_cmd(self, range_cb):
        return runOnce(lambda: self.set_pos_from_range(range_cb))


    def angulator_off_cmd (self):
        return runOnce(
           lambda: self.angulatorMotor.set_control(MotionMagicVoltage(position=0 + self.encoder_offset))  
        )


    def apply_encoder_config_with_retries(self, config):
        for i in range(5):
            result = self.angulatorEncoder.configurator.apply(config)
            if result == StatusCode.OK:
                break


    def zero_encoder(self):
        # Wait for a new reading after applying the offset.
        angulator_offset = self.angulatorEncoder.get_absolute_position()
        angulator_offset.wait_for_update(1)

        self.encoder_offset = angulator_offset.value
        
        self.angulatorMotorConfig.software_limit_switch.forward_soft_limit_threshold = constants.kAngulatorForwardSoftLimitRot + self.encoder_offset
        self.angulatorMotorConfig.software_limit_switch.reverse_soft_limit_threshold = constants.kAngulatorReverseSoftLimitRot + self.encoder_offset
        self.angulatorMotor.configurator.apply(self.angulatorMotorConfig)
        wpilib.Preferences.setDouble("ANGULATOR_OFFSET", angulator_offset.value)
        SmartDashboard.putNumber("Angulator Absolute", angulator_offset.value)


    def zero_angulator_encoder_cmd(self):
        return runOnce(self.zero_encoder)
    
    def wait_for_angulator_on_target(self):
        return WaitUntilCommand(lambda: math.fabs(self.get_error()) < 0.01)


    def periodic(self):
       SmartDashboard.putNumber("Angulator Position",self.angulatorEncoder.get_absolute_position().value - self.encoder_offset)
       SmartDashboard.putNumber("Angulator Error", self.get_error())
       

        

    


