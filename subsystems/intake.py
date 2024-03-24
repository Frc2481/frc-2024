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

from wpilib import DigitalInput, SmartDashboard


class IntakeSubsystem(commands2.SubsystemBase):

    def __init__(self):
        super().__init__()
        
        self.horizontalMotor = TalonFX(constants.kIntakeHorizontalMotorCANID, "2481")
        self.horizontalMotorConfig = TalonFXConfiguration()
        self.horizontalMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.horizontalMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.horizontalMotor.configurator.apply(self.horizontalMotorConfig)
        
        self.verticalMotor = TalonFX(constants.kIntakeVerticalMotorCANID, "2481")
        self.verticalMotorConfig = TalonFXConfiguration()
        self.verticalMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
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


    def intake_off(self):
        self.horizontalMotor.set_control(VoltageOut(0))
        self.verticalMotor.set_control(VoltageOut(0))


    def intake_off_cmd(self):
        return commands2.cmd.runOnce(
            lambda: self.intake_off())
        
    def periodic(self):
        SmartDashboard.putNumber("intake horizontal duty cycle", self.horizontalMotor.get_duty_cycle().value)
        SmartDashboard.putNumber("intake vertical duty cycle", self.verticalMotor.get_duty_cycle().value)
        SmartDashboard.putNumber("horizontal intake current", self.horizontalMotor.get_supply_current().value)
        SmartDashboard.putNumber("vertical intake current", self.verticalMotor.get_supply_current().value)

