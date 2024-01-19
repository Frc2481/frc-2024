import math
import wpilib

import commands2
import commands2.cmd

import constants

import phoenix6
from phoenix6.signals.spn_enums import *

class FeederSubsystem(object):

    def __init__(self):
        super().__init__()

        self.feederMotor = phoenix6.TalonFX(constants.kFeederMotorCANID)

        self.feederMotorConfig = phoenix6.TalonFXConfiguration()
        self.feederMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.feederMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.feederMotor.configurator.apply(self.verticalMotorConfig)

    def feeder_on_cmd (self):
        self.feederMotor.set_control(phoenix6.VoltageOut(constants.kFeederSpeed * 12))

    def feeder_off_cmd (self):
        self.feederMotor.set_control(phoenix6.VoltageOut(0))
  

