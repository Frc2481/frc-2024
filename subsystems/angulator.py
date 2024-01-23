import math

import wpilib
import commands2
import commands2.cmd
from commands2 import *
from commands2.cmd import * 

import constants

import phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityTorqueCurrentFOC, VoltageOut, VelocityVoltage
from phoenix6.signals.spn_enums import *



class AngulatorSubsystem(commands2.SubsystemBase):
    
    def __init__(self):
        super().__init__()

        self.angulatorMotor = TalonFX(constants.kAngulatorMotorCANID)

        self.angulatorMotorConfig = TalonFXConfiguration()
        self.angulatorMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.angulatorMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.angulatorMotor.configurator.apply(self.angulatorMotorConfig)

    def angulator_up_cmd (self, angulator_speed_rps = constants.kAngulatorSpeedRPS):
       return runOnce(
           # Use this one on the robot.
           # lambda:  self.angulatorMotor.set_control(VelocityTorqueCurrentFOC(angulator_speed_rps))
           
           # Using this one so SIM works :(
            lambda:  self.angulatorMotor.set_control(VelocityVoltage(angulator_speed_rps))
        )
    
    def angulator_down_cmd (self, angulator_speed_rps = constants.kAngulatorSpeedRPS):
       return runOnce(
           # Use this one on the robot.
           # lambda:  self.angulatorMotor.set_control(VelocityTorqueCurrentFOC(angulator_speed_rps))
           
           # Using this one so SIM works :(
            lambda:  self.angulatorMotor.set_control(VelocityVoltage(-angulator_speed_rps))
        )


    def angulator_off_cmd (self):
        return runOnce(
           lambda: self.angulatorMotor.set_control(VoltageOut(0))
        )
        

    


