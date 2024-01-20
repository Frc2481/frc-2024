import math
import wpilib

import commands2
import commands2.cmd

import constants

import phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.signals.spn_enums import *

class ShooterSubsystem(object):

    def __init__(self):
        super().__init__()

        self.shooterMotor = TalonFX(constants.kShooterMotorCANID)

        self.shooterMotorConfig = TalonFXConfiguration()
        self.shooterMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.shooterMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.shooterMotor.configurator.apply(self.shooterMotorConfig)

    def shooter_on_cmd(self):
        self.shooterMotor.set_control(VoltageOut(constants.kShooterSpeed * 12))
    def shooter_off_cmd(self):
        self.shooterMotor.set_control(VoltageOut(0))  
