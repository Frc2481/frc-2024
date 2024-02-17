import math

import wpilib
from commands2 import *
from commands2.cmd import * 

import constants

import phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityTorqueCurrentFOC, VoltageOut, VelocityVoltage
from phoenix6.signals.spn_enums import *

class ShooterSubsystem(object):

    def __init__(self):
        super().__init__()

        self.shooterMotor = TalonFX(constants.kShooterMotorCANID, "2481")

        self.shooterMotorConfig = TalonFXConfiguration()
        self.shooterMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.shooterMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.shooterMotorConfig.slot0.k_p = constants.kShooterP
        self.shooterMotorConfig.slot0.k_i = constants.kShooterI
        self.shooterMotorConfig.slot0.k_d = constants.kShooterD
        self.shooterMotorConfig.slot0.k_v = constants.kShooterV
        self.shooterMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kShooterCruiseVelocity
        self.shooterMotorConfig.motion_magic.motion_magic_acceleration = constants.kShooterAcceleration
        self.shooterMotorConfig.motion_magic.motion_magic_jerk = constants.kShooterJerk
        self.shooterMotorConfig.torque_current.peak_forward_torque_current

        self.shooterMotor.configurator.apply(self.shooterMotorConfig)

    def shooter_on_cmd(self, shooter_speed_rps = constants.kShooterSpeedRPS):
        return runOnce(
            # Use this one on the robot.
            # lambda: self.shooterMotor.set_control(VelocityTorqueCurrentFOC(shooter_speed_rps))

            # Using this one so SIM works :(
            lambda: self.shooterMotor.set_control(VelocityVoltage(shooter_speed_rps))
        )
        
    def shooter_off_cmd(self):
        return runOnce(
            lambda: self.shooterMotor.set_control(VoltageOut(0))
        )
    
    def shooter_to_arm_cmd(self, shooter_to_arm_speed_rps = constants.kShooterToArmSpeedRPS):
        return runOnce(
            lambda: self.shooterMotor.set_control(VelocityVoltage(shooter_to_arm_speed_rps))
        )
          
