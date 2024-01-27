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

from wpilib import DigitalInput

class FeederSubsystem(object):

    def __init__(self):
        super().__init__()

        self.feederBeambreak = DigitalInput(constants.kFeederBeambreakPort)
        
        self.feederMotor = TalonFX(constants.kFeederMotorCANID)

        self.feederMotorConfig = TalonFXConfiguration()
        self.feederMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.feederMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.feederMotorConfig.slot0.k_p = constants.kFeederP
        self.feederMotorConfig.slot0.k_i = constants.kFeederI
        self.feederMotorConfig.slot0.k_d = constants.kFeederD
        self.feederMotorConfig.slot0.k_v = constants.kFeederV
        self.feederMotorConfig.torque_current.peak_forward_torque_current
        self.feederMotor.configurator.apply(self.feederMotorConfig)


    def feeder_on_cmd (self, feeder_speed_rps = constants.kFeederSpeedRPS):
       return runOnce(
           # Use this one on the robot.
           # lambda:  self.feederMotor.set_control(VelocityTorqueCurrentFOC(feeder_speed_rps))
           
           # Using this one so SIM works :(
            lambda:  self.feederMotor.set_control(VelocityVoltage(feeder_speed_rps))
        )


    def feeder_off_cmd (self):
        return runOnce(
           lambda: self.feederMotor.set_control(VoltageOut(0))
        )
    
    def has_game_piece(self) -> bool:
        # TODO beam break
        return self.feederBeambreak.get()
       
    
    def feeder_piece_ejected(self) -> bool:
        # TODO beam break
        return not self.feederBeambreak.get()
        
    