import math
import wpilib



from commands2 import *
from commands2.cmd import *

import constants

import phoenix6
from phoenix6.hardware import TalonFX

from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityTorqueCurrentFOC, VoltageOut, VelocityVoltage, DutyCycleOut
from phoenix6.signals.spn_enums import *
from wpilib import SmartDashboard


class FeederSubsystem(object):

    def __init__(self):
        super().__init__()        
        self.feederMotor = TalonFX(constants.kFeederMotorCANID, "2481")

        self.feederMotorConfig = TalonFXConfiguration()
        self.feederMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.feederMotorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.feederMotorConfig.slot0.k_p = constants.kFeederP
        self.feederMotorConfig.slot0.k_i = constants.kFeederI
        self.feederMotorConfig.slot0.k_d = constants.kFeederD
        self.feederMotorConfig.slot0.k_v = constants.kFeederV
        self.feederMotorConfig.torque_current.peak_forward_torque_current
        self.feederMotor.configurator.apply(self.feederMotorConfig)
        
        self.voltage_control_signal = VoltageOut(0)
        self.off_signal = VoltageOut(0)


    def feeder_on_cmd (self, duty):
       return runOnce(
            lambda:  self.feederMotor.set_control((self.voltage_control_signal.with_output(duty * 12.0)))
            )


    def feeder_off_cmd (self):
        return runOnce(
           lambda: self.feederMotor.set_control(self.off_signal)
        )
    

    