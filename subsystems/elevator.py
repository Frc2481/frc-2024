import wpilib

from commands2 import *
from commands2.cmd import *

import constants
from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX

import phoenix6
from phoenix6.signals.spn_enums import *

class ElevatorSubsystem():
    
    def __init__(self):
        super().__init__()

        self.elevatorMotor = TalonFX(constants.kElevatorMotorCANID)

        self.elevatorMotorConfig = TalonFXConfiguration()
        self.elevatorMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.elevatorMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.elevatorMotor.configurator.apply(self.elevatorMotorConfig)


