"""Intake Submodule """
import commands2
import commands2.cmd

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.signals import spn_enums

from wpilib import DigitalInput

import constants

class IntakeSubsystem(object):
    """Intake Subsystem Class"""
    def __init__(self):
        super().__init__()

        self.intake_beam_break = DigitalInput(constants.INTAKE_BEAM_BREAK_PORT)

        self.horizontal_motor = TalonFX(constants.INTAKE_HORIZONTAL_MOTOR_CAN_ID)
        self.vertical_motor = TalonFX(constants.INTAKE_VERTICAL_MOTOR_CAN_ID)
        self.horizontal_motor_config = TalonFXConfiguration()
        self.horizontal_motor_config.motor_output.neutral_mode = spn_enums.NeutralModeValue.BRAKE
        self.horizontal_motor_config.motor_output.inverted = \
            spn_enums.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.horizontal_motor.configurator.apply(self.horizontal_motor_config)

        self.vertical_motor_config = TalonFXConfiguration()
        self.vertical_motor_config.motor_output.neutral_mode = spn_enums.NeutralModeValue.BRAKE
        self.vertical_motor_config.motor_output.inverted = \
            spn_enums.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.vertical_motor.configurator.apply(self.vertical_motor_config)

    def set_intake(self, horizontal=None, vertical=None):
        """Set Intake Params""" 
        if horizontal is not None:
            self.horizontal_motor.set_control(VoltageOut(horizontal * 12.0))
        if vertical is not None:
            self.vertical_motor.set_control(VoltageOut(vertical * 12.0))

    def set_intake_cmd(self, horizontal=None, vertical=None):
        """command for setting intake speeds.""" 
        return commands2.cmd.runOnce(
            lambda: self.set_intake(horizontal, vertical)
        )

    def has_game_piece(self) -> bool:
        """Read intake beam break to see if we have a note."""
        return self.intake_beam_break.get()

    def game_piece_ejected(self) -> bool:
        """returns true of game piece is no longer on board."""
        return not self.intake_beam_break.get()
