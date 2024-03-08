import math
import wpilib
from wpilib import DoubleSolenoid

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import *
from phoenix6.controls import MotionMagicVoltage, VelocityTorqueCurrentFOC, VoltageOut
from phoenix6.configs import TalonFXConfiguration
from wpilib import SmartDashboard, DigitalInput

import constants
from commands2 import *
import commands2.cmd
from commands2.cmd import *
from commands2 import InstantCommand

class ArmSubsystem(Subsystem):

    def __init__(self):
               
        self.armMotor = TalonFX(constants.kArmMotorCANID, "2481")
        self.climberSolenoid = DoubleSolenoid(
            # constants.kGripperSolenoidModule,\[]
            moduleType = wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel = constants.kClimberDoubleSolenoidForwardPort,
            reverseChannel = constants.kClimberDoubleSolenoidReversePort
        )  
        self.armMotorConfig = TalonFXConfiguration()
        self.armMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.armMotorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.armMotorConfig.slot0.k_p = constants.kArmP
        self.armMotorConfig.slot0.k_i = constants.kArmI 
        self.armMotorConfig.slot0.k_d = constants.kArmD
        self.armMotorConfig.slot0.k_v = constants.kArmV
        self.armMotorConfig.slot0.k_a = constants.kArmA
        self.armMotorConfig.slot0.k_s = constants.kArmS
        self.armMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kArmCruiseVelocity
        self.armMotorConfig.motion_magic.motion_magic_acceleration = constants.kArmAcceleration
        
        self.armMotorConfig.software_limit_switch.forward_soft_limit_threshold = 36
        self.armMotorConfig.software_limit_switch.reverse_soft_limit_threshold = 0
        self.armMotorConfig.software_limit_switch.forward_soft_limit_enable = True
        self.armMotorConfig.software_limit_switch.reverse_soft_limit_enable =  True
        
        self.armMotor.configurator.apply(self.armMotorConfig)
        self.zero_arm_switch =  DigitalInput(constants.kArmZero)
        self.setpoint = 0
        
        self.gripperSolenoid = DoubleSolenoid(
            # constants.kGripperSolenoidModule,
            moduleType = wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel = constants.kGripperDoubleSolenoidForwardPort,
            reverseChannel = constants.kGripperDoubleSolenoidReversePort
        ) 


    def get_error(self):
        return self.setpoint - self.armMotor.get_position().value


    def set_arm_position(self, position):
        self.setpoint = position
        self.armMotor.set_control(MotionMagicVoltage(position=position))


    def arm_set_pos_cmd (self, arm_position):
        return FunctionalCommand(
            lambda: self.set_arm_position(arm_position),
            lambda: None,
            lambda interrupted: None,
            lambda: math.fabs(self.get_error()) < 1,
            self
        )       


    def arm_climb_pos_cmd (self, arm_position = constants.kArmClimbPosition):
           return runOnce(
            lambda:  self.armMotor.set_control(MotionMagicVoltage(0).with_position(arm_position))
            
        )    


    def arm_score_pos_cmd (self, arm_position = constants.kArmScorePosition):
           return runOnce(
            lambda:  self.armMotor.set_control(MotionMagicVoltage(0).with_position(arm_position))
            
        )       


    def arm_stow_pos_cmd (self, arm_position = constants.kArmDownPosition):
        return sequence(    
            self.gripper_open_cmd(),
            InstantCommand (lambda:self.armMotor.set_control(MotionMagicVoltage(0).with_position(arm_position)))
        )           


    def arm_pickup_pos_cmd (self, arm_position = constants.kArmPickupPosition):
           return self.arm_set_pos_cmd(arm_position)         


    def gripper_open_cmd(self):
        return runOnce(
            lambda: self.gripperSolenoid.set(DoubleSolenoid.Value.kForward)
        ) 


    def close_gripper_safe(self):
        if self.armMotor.get_position().value > 7: # Position at which we can hit electronics.
            self.gripperSolenoid.set(DoubleSolenoid.Value.kReverse)


    def gripper_close_cmd(self):
            return runOnce(self.close_gripper_safe)


    def periodic(self):
        SmartDashboard.putNumber("Arm Position", self.armMotor.get_position().value)
        if self.zero_arm_switch.get() == 0:
            if self.armMotor.get_position().value == 0:
                self.armMotor.set_position(0) 
    
    
    # Climber up
    def kill_the_beast(self):
        if self.armMotor.get_position().value >= (constants.kArmClimbPosition * .55) and self.gripperSolenoid.Value.kReverse:
            self.climberSolenoid.set(DoubleSolenoid.Value.kReverse)

    
    # Climber down    
    def batman_grapling_hook(self):
        if self.armMotor.get_position().value >= (constants.kArmClimbPosition * .55) and self.gripperSolenoid.Value.kReverse:
            self.climberSolenoid.set(DoubleSolenoid.Value.kForward)


    def kill_the_beast_cmd(self):
        return runOnce(lambda: self.kill_the_beast())

    
    def batman_grappling_hook_cmd(self):
        return runOnce(lambda: self.batman_grapling_hook())