import math
import wpilib

import commands2
import commands2.cmd

import constants

import phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut, DutyCycleOut
from phoenix6.signals.spn_enums import *

from wpilib import DigitalInput


class IntakeSubsystem(commands2.SubsystemBase):

    def __init__(self):
        super().__init__()
        
        self.intakeBeambreak = DigitalInput(constants.kIntakeBeambreakPort)

        self.horizontalMotor = TalonFX(constants.kIntakeHorizontalMotorCANID, "2481")
        self.verticalMotor = TalonFX(constants.kIntakeVerticalMotorCANID, "2481")
        self.horizontalMotorConfig = TalonFXConfiguration()
        self.horizontalMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.horizontalMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.horizontalMotor.configurator.apply(self.horizontalMotorConfig)

        self.verticalMotorConfig = TalonFXConfiguration()
        self.verticalMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.verticalMotorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.verticalMotor.configurator.apply(self.verticalMotorConfig)

    def set_intake(self, horizontal=None, vertical=None):

        if horizontal is not None:
            self.horizontalMotor.set_control(DutyCycleOut(horizontal))
        if vertical is not None:
            self.verticalMotor.set_control(DutyCycleOut(vertical))

    def set_intake_cmd(self, horizontal=None, vertical=None):

        return commands2.cmd.runOnce(
            lambda: self.set_intake(horizontal, vertical)
        )
    
    def has_game_piece(self) -> bool:
        # TODO beam break
        return self.intakeBeambreak.get()
    
    def intake_piece_ejected(self) -> bool:
        # TODO beam break
        return not self.intakeBeambreak.get()
    