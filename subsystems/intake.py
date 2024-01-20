import math
import wpilib

import commands2
import commands2.cmd

import constants

import phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration 
from phoenix6.signals.spn_enums import *

class IntakeSubsystem(object):

    def __init__(self):
        super().__init__()

        self.horizontalMotor = TalonFX(constants.kIntakeHorizontalMotorCANID)
        self.verticalMotor = TalonFX(constants.kIntakeVerticalMotorCANID)
        self.horizontalMotorConfig = TalonFXConfiguration()
        self.horizontalMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.horizontalMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.horizontalMotor.configurator.apply(self.horizontalMotorConfig)

        self.verticalMotorConfig = TalonFXConfiguration()
        self.verticalMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.verticalMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.verticalMotor.configurator.apply(self.verticalMotorConfig)

    def set_intake(self, horizontal=None, vertical=None):
        
        if horizontal is not None:
            self.horizontalMotor.set_control(phoenix6.VoltageOut(horizontal * 12.0))
        if vertical is not None:
            self.verticalMotor.set_control(phoenix6.VoltageOut(vertical * 12.0))

    def set_intake_cmd(self, horizontal=None, vertical=None):
        
        return runOnce(
            lambda: self.set_rollers(horizontal, vertical)
        )