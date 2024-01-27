import math

import wpilib
import commands2
import commands2.cmd
from commands2 import *
from commands2.cmd import * 
import ntcore 

import constants

import phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityTorqueCurrentFOC, VoltageOut, VelocityVoltage, MotionMagicVoltage
from phoenix6.signals.spn_enums import *



class AngulatorSubsystem(commands2.SubsystemBase):
    
    def __init__(self):
        super().__init__()

        self.angulatorMotor = TalonFX(constants.kAngulatorMotorCANID)

        self.angulatorMotorConfig = TalonFXConfiguration()
        self.angulatorMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.angulatorMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.angulatorMotorConfig.slot0.k_p = constants.kAngulatorP
        self.angulatorMotorConfig.slot0.k_i = constants.kAngulatorI
        self.angulatorMotorConfig.slot0.k_d = constants.kAngulatorD
        self.angulatorMotorConfig.slot0.k_v = constants.kAngulatorV
        self.angulatorMotorConfig.slot0.k_a = constants.kAngulatorA
        self.angulatorMotorConfig.slot0.k_s = constants.kAngulatorS
        self.angulatorMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kAngulatorCruiseVelocity
        self.angulatorMotorConfig.motion_magic.motion_magic_acceleration = constants.kAngulatorAcceleration
        self.angulatorMotorConfig.motion_magic.motion_magic_jerk = constants.kAngulatorJerk
        self.angulatorMotorConfig.feedback.sensor_to_mechanism_ratio = constants.kAngulatorGearReduction
        self.__sd = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")

        self.angulatorMotor.configurator.apply(self.angulatorMotorConfig)

    def angulator_up_cmd (self, angulator_position = constants.kAngulatorUpPosition):
       return runOnce(
           # Use this one on the robot.
           # lambda:  self.angulatorMotor.set_control(VelocityTorqueCurrentFOC(angulator_speed_rps))
           
           # Using this one so SIM works :(
            lambda:  self.angulatorMotor.set_control(MotionMagicVoltage(0).with_position(angulator_position))
        )
    
    def angulator_down_cmd (self, angulator_position = constants.kAngulatorDownPosition):
       return runOnce(
           # Use this one on the robot.
           # lambda:  self.angulatorMotor.set_control(VelocityTorqueCurrentFOC(angulator_speed_rps))
           
           # Using this one so SIM works :(
            lambda:  self.angulatorMotor.set_control(MotionMagicVoltage(0).with_position(angulator_position))
        )


    def angulator_off_cmd (self):
        return runOnce(
           lambda: self.angulatorMotor.set_control(VoltageOut(0))
        )
    
    def periodic(self):
       self.__sd.putNumber("Angulator Current", self.angulatorMotor.get_supply_current().value)
       self.__sd.putNumber("Angulator Position",self.angulatorMotor.get_position().value)
       self.__sd.putNumber("Angulator Velocity", self.angulatorMotor.get_velocity().value)

        

    


