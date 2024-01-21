import math

import wpilib
from commands2 import *
from commands2.cmd import * 

import constants

import phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityTorqueCurrentFOC, VoltageOut
from phoenix6.signals.spn_enums import *

class ShooterSubsystem(object):

    def __init__(self):
        super().__init__()

        self.shooterMotor = TalonFX(constants.SHOOTER_MOTOR_CAN_ID)

        self.shooterMotorConfig = TalonFXConfiguration()
        self.shooterMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.shooterMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.shooterMotor.configurator.apply(self.shooterMotorConfig)

        self.shooterMotorConfig.slot0.k_p = constants.SHOOTER_P
        self.shooterMotorConfig.slot0.k_i = constants.SHOOTER_I
        self.shooterMotorConfig.slot0.k_d = constants.SHOOTER_D
        self.shooterMotorConfig.slot0.k_f = constants.SHOOTER_F
        self.shooterMotorConfig.torque_current.peak_forward_torque_current

    def shooter_on_cmd(self, shooter_speed_rps = constants.SHOOTER_SPEED_RPS):
        return runOnce(
            lambda: self.shooterMotor.set_control(VelocityTorqueCurrentFOC(shooter_speed_rps))
        )
        
    def shooter_off_cmd(self):
        return runOnce(
            lambda: self.shooterMotor.set_control(VoltageOut(0))
        )
          
