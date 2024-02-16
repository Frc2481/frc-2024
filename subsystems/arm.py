import math
import wpilib
from wpilib import DoubleSolenoid

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import *
from phoenix6.controls import MotionMagicVoltage, VelocityTorqueCurrentFOC
from phoenix6.configs import TalonFXConfiguration

import constants

import commands2
import commands2.cmd
from commands2.cmd import *
from commands2 import InstantCommand


class ArmSubsystem(object):

    def __init__(self,):
               
        self.armMotor = TalonFX(constants.kArmMotorCANID)
        
        self.armMotorConfig = TalonFXConfiguration()
        self.armMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.armMotorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.armMotorConfig.slot0.k_p = constants.kArmP
        self.armMotorConfig.slot0.k_i = constants.kArmI 
        self.armMotorConfig.slot0.k_d = constants.kArmD
        self.armMotorConfig.slot0.k_v = constants.kArmV
        self.armMotorConfig.slot0.k_a = constants.kArmA
        self.armMotorConfig.slot0.k_s = constants.kArmS
        self.armMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kArmCruiseVelocity
        self.armMotorConfig.motion_magic.motion_magic_acceleration = constants.kArmAcceleration
        self.armMotorConfig.feedback.sensor_to_mechanism_ratio = constants.kArmGearReduction
        self.armMotor.configurator.apply(self.armMotorConfig)
        
                       
        self.armMotorFollower = TalonFX(constants.kArmMotorFollowerCANID)
        
        self.armMotorFollowerConfig = TalonFXConfiguration()
        self.armMotorFollowerConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.armMotorFollowerConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.armMotorFollowerConfig.slot0.k_p = constants.kArmP
        self.armMotorFollowerConfig.slot0.k_i = constants.kArmI 
        self.armMotorFollowerConfig.slot0.k_d = constants.kArmD
        self.armMotorFollowerConfig.slot0.k_v = constants.kArmV
        self.armMotorFollowerConfig.slot0.k_a = constants.kArmA
        self.armMotorFollowerConfig.slot0.k_s = constants.kArmS
        self.armMotorFollowerConfig.motion_magic.motion_magic_cruise_velocity = constants.kArmCruiseVelocity
        self.armMotorFollowerConfig.motion_magic.motion_magic_acceleration = constants.kArmAcceleration
        self.armMotorFollowerConfig.feedback.sensor_to_mechanism_ratio = constants.kArmGearReduction
        self.armMotorFollower.configurator.apply(self.armMotorFollowerConfig)
        
    #     self.armSolenoid = DoubleSolenoid(
    #         constants.kArmSolenoidModule,
    #         moduleType = wpilib.PneumaticsModuleType.CTREPCM,
    #         forwardChannel = constants.kArmDoubleSolenoidForwardPort,
    #         reverseChannel = constants.kArmDoubleSolenoidReversePort
    #     )        
        
    #     self.armSolenoid2 = DoubleSolenoid(
    #         constants.kArmSolenoidModule,
    #         moduleType = wpilib.PneumaticsModuleType.CTREPCM,
    #         forwardChannel = constants.kArmDoubleSolenoid2ForwardPort,
    #         reverseChannel = constants.kArmDoubleSolenoid2ReversePort
    #    )        
        
        self.gripperSolenoid = DoubleSolenoid(
            constants.kGripperSolenoidModule,
            moduleType = wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel = constants.kGripperDoubleSolenoidForwardPort,
            reverseChannel = constants.kGripperDoubleSolenoidReversePort
         )        

   
            
    def arm_up_cmd (self, arm_position = constants.kArmUpPosition):
           return runOnce(
           # Use this one on the robot.
           #lambda:  self.armMotor.set_control(VelocityTorqueCurrentFOC(constants.kArmSpeedRPS))
           
           # Using this one so SIM works :(
            lambda:  self.armMotor.set_control(MotionMagicVoltage().with_position(arm_position / 360.0 * 5))
            #lambda:  self.armMotorFollower
        )       
        
    def arm_down_cmd (self, arm_position = constants.kArmDownPosition):
           return runOnce(
           # Use this one on the robot.
           # lambda:  self.angulatorMotor.set_control(VelocityTorqueCurrentFOC(constants.kArmSpeedRPS))
           
           # Using this one so SIM works :(
            lambda:  self.armMotor.set_control(MotionMagicVoltage(0).with_position(arm_position / 360.0 * 5))
        )
           
    def arm_stow_cmd (self, arm_position = constants.kArmStowPosition):
           return runOnce(
           # Use this one on the robot.
           # lambda:  self.angulatorMotor.set_control(VelocityTorqueCurrentFOC(constants.kArmSpeedRPS))
           
           # Using this one so SIM works :(
            lambda:  self.armMotor.set_control(MotionMagicVoltage(0).with_position(arm_position / 360.0 * 5))
        )           
        
    def gripper_open_cmd(self):
        return runOnce(
            lambda: self.gripperSolenoid.set(DoubleSolenoid.Value.kForward)
        ) 
        
    def gripper_close_cmd(self):
            return runOnce(
            lambda: self.gripperSolenoid.set(DoubleSolenoid.Value.kReverse)
        )