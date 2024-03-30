import math
import wpimath.units
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.geometry import Rotation2d

from phoenix6.controls import VelocityVoltage, MotionMagicVoltage, VoltageOut, DutyCycleOut

from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.cancoder_configs import CANcoderConfiguration 

from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware import TalonFX
from phoenix6.signals.spn_enums import *

import constants
from utils import *

class SwerveModule(object):
    
    def __init__(self, driveCANID, steerCANID, steerCANCoderID, steerInverted):
        self.id = driveCANID

        self.driveMotor = TalonFX(driveCANID, "2481")
        self.steerMotor = TalonFX(steerCANID, "2481")
        self.steerEncoder = CANcoder(steerCANCoderID, "2481")

        self.driveMotorConfig = TalonFXConfiguration()
        self.driveMotorConfig.current_limits.stator_current_limit = 80
        
        self.driveMotorConfig.current_limits.stator_current_limit_enable
        self.driveMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.driveMotorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.driveMotorConfig.slot0.k_p = constants.kdriveP
        self.driveMotorConfig.slot0.k_i = constants.kdriveI 
        self.driveMotorConfig.slot0.k_d = constants.kdriveD
        self.driveMotorConfig.slot0.k_v = constants.kdriveV
        self.driveMotorConfig.slot0.k_a = constants.kdriveA
        self.driveMotorConfig.slot0.k_s = constants.kdriveS
        self.driveMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kSwerveSteerCruiseVelocity
        self.driveMotorConfig.motion_magic.motion_magic_acceleration = constants.kSwerveSteerAcceleration 
        self.driveMotorConfig.feedback.sensor_to_mechanism_ratio = constants.kSwerveReductionDrive
        self.driveMotor.configurator.apply(self.driveMotorConfig)
        
        self.steerMotorConfig = TalonFXConfiguration()
        self.steerMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.steerMotorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if steerInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.steerMotorConfig.motor_output.peak_forward_duty_cycle = 0.2
        self.steerMotorConfig.motor_output.peak_reverse_duty_cycle = -0.2
        self.steerMotorConfig.slot0.k_p = constants.ksteerP
        self.steerMotorConfig.slot0.k_i = constants.ksteerI
        self.steerMotorConfig.slot0.k_d = constants.ksteerD
        self.steerMotorConfig.slot0.k_v = constants.ksteerV
        self.steerMotorConfig.slot0.k_a = constants.ksteerA
        self.steerMotorConfig.slot0.k_s = constants.ksteerS
        self.steerMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kSwerveSteerCruiseVelocity
        self.steerMotorConfig.motion_magic.motion_magic_acceleration = constants.kSwerveSteerAcceleration
        self.steerMotorConfig.motion_magic.motion_magic_jerk = constants.kSwerveSteerJerk
        self.steerMotorConfig.closed_loop_general.continuous_wrap = True

        self.steerMotorConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        self.steerMotorConfig.feedback.feedback_remote_sensor_id = steerCANCoderID 
        self.steerMotorConfig.feedback.sensor_to_mechanism_ratio = 1.0
        self.steerMotorConfig.feedback.rotor_to_sensor_ratio = constants.kSwerveReductionSteer
        self.steerMotor.configurator.apply(self.steerMotorConfig)

        self.canCoderConfig = CANcoderConfiguration()
        self.canCoderConfig.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        self.canCoderConfig.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        self.canCoderConfig.magnet_sensor.magnet_offset = 0.4
        self.steerEncoder.configurator.apply(self.canCoderConfig)

        self.wheel_circumference = 0
        


    def zero_steer_encoder(self):
        print("Zero Encoder Start")
        #zeros the offset so we dont have to deal with the previous one
        self.canCoderConfig.magnet_sensor.magnet_offset = 0
        self.steerEncoder.configurator.apply(self.canCoderConfig)
        
        #Wait for fresh data after offset
        steer_offset = self.steerEncoder.get_absolute_position()
        steer_offset.wait_for_update(1)
        self.canCoderConfig.magnet_sensor.magnet_offset = -steer_offset.value
        self.steerEncoder.configurator.apply(self.canCoderConfig)
        print("Zero Encoder Finish")
        return -steer_offset.value
        
    def set_steer_offset(self, steer_offset:float):
        self.canCoderConfig.magnet_sensor.magnet_offset = steer_offset
        self.steerEncoder.configurator.apply(self.canCoderConfig)
       
    def distance(self):
        return wpimath.units.inchesToMeters(self.driveMotor.get_position().value * self.wheel_circumference)
    
    
    def angle(self):
        return Rotation2d(self.steerEncoder.get_absolute_position().value * 2 * math.pi)
    
    def set_state(self, state: SwerveModuleState, voltage_only):
        state = SwerveModuleState.optimize(state, self.angle())
        if voltage_only:
            self.driveMotor.set_control(DutyCycleOut(state.speed / constants.kDriveMaxSpeed, True))
        else:
            self.driveMotor.set_control(VelocityVoltage(wpimath.units.metersToInches(state.speed) / self.wheel_circumference))        
        
        if abs(state.speed) < 0.05:
            self.steerMotor.set_control(VoltageOut(0.0)) 
        else:   
            self.steerMotor.set_control(MotionMagicVoltage(state.angle.degrees() / 360))
    
    def get_position(self):
        return SwerveModulePosition(
            #negative sign is to fix inverted Odometry
            distance=self.distance(),
            angle=self.angle()
        )
   
    def get_state(self): 
        return SwerveModuleState(
            speed=wpimath.units.inchesToMeters(self.driveMotor.get_velocity().value * (self.wheel_circumference)),
            angle=self.angle()
        )
    
    def zero_drive_encoder(self):
       self.driveMotor.set_position(0)
    
    def get_voltage(self):
        return self.driveMotor.get_motor_voltage().value
