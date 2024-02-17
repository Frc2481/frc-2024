import math
import wpilib
from wpilib import DoubleSolenoid

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import *
from phoenix6.controls import MotionMagicVoltage, VelocityTorqueCurrentFOC, VoltageOut
from phoenix6.configs import TalonFXConfiguration
from wpilib import SmartDashboard

import constants
from commands2 import *
import commands2.cmd
from commands2.cmd import *
from commands2 import InstantCommand


class ArmSubsystem(Subsystem):

    def __init__(self,):
               
        self.armMotor = TalonFX(constants.kArmMotorCANID, "2481")
        
        self.armMotorConfig = TalonFXConfiguration()
        self.armMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.armMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.armMotorConfig.slot0.k_p = constants.kArmP
        self.armMotorConfig.slot0.k_i = constants.kArmI 
        self.armMotorConfig.slot0.k_d = constants.kArmD
        self.armMotorConfig.slot0.k_v = constants.kArmV
        self.armMotorConfig.slot0.k_a = constants.kArmA
        self.armMotorConfig.slot0.k_s = constants.kArmS
        self.armMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kArmCruiseVelocity
        self.armMotorConfig.motion_magic.motion_magic_acceleration = constants.kArmAcceleration
        
        self.armMotorConfig.software_limit_switch.forward_soft_limit_threshold = 35.44
        self.armMotorConfig.software_limit_switch.reverse_soft_limit_threshold = 0
        self.armMotorConfig.software_limit_switch.forward_soft_limit_enable = True
        self.armMotorConfig.software_limit_switch.reverse_soft_limit_enable =  True
        
        self.armMotor.configurator.apply(self.armMotorConfig)
        
    
        self.gripperSolenoid = DoubleSolenoid(
            # constants.kGripperSolenoidModule,
            moduleType = wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel = constants.kGripperDoubleSolenoidForwardPort,
            reverseChannel = constants.kGripperDoubleSolenoidReversePort
         )        

   
            
    def arm_up_cmd (self, arm_position = constants.kArmUpPosition):
           return runOnce(
            lambda:  self.armMotor.set_control(MotionMagicVoltage(0).with_position(arm_position))
            
        )       
        
    def arm_down_cmd (self, arm_position = constants.kArmDownPosition):
           return runOnce(
            lambda:  self.armMotor.set_control(MotionMagicVoltage(0).with_position(arm_position))
        )
           
    def arm_pickup_position_cmd (self, arm_position = constants.kArmStowPosition):
           return runOnce(
            lambda:  self.armMotor.set_control(MotionMagicVoltage(0).with_position(arm_position))
        )
           
    def climb_cmd (self):
        self.armMotor.set_control(VoltageOut(3))                  
        
    def gripper_open_cmd(self):
        return runOnce(
            lambda: self.gripperSolenoid.set(DoubleSolenoid.Value.kReverse)
        ) 
        
    def gripper_close_cmd(self):
            return runOnce(
            lambda: self.gripperSolenoid.set(DoubleSolenoid.Value.kForward)
        )
    def periodic(self):
        SmartDashboard.putNumber("Arm Position", self.armMotor.get_position().value)        