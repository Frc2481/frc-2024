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

class FeederSubsystem(object):

    def __init__(self):
        super().__init__()

        self.feederMotor = TalonFX(constants.kFeederMotorCANID)

        self.feederMotorConfig = TalonFXConfiguration()
        self.feederMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.feederMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.feederMotor.configurator.apply(self.feederMotorConfig)

    def feeder_on_cmd (self):
        self.feederMotor.set_control(VoltageOut(constants.kFeederSpeed * 12))

    def feeder_off_cmd (self):
        self.feederMotor.set_control(VoltageOut(0))
  

