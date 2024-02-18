import math

from wpilib import SmartDashboard
import wpilib
import commands2
import commands2.cmd
from commands2 import *
from commands2.cmd import * 
import ntcore 
import constants
import phoenix5 #robotpy_ctre
from phoenix5 import *


class AngulatorSubsystem(commands2.SubsystemBase):
    def enc_ticks_to_rot(self, ticks):
        return ticks / 4096
    
    def rot_to_enc_ticks(self, rotations):
        return rotations * 4096
    
    def enc_ticks_per_100_ms_to_rps(self, ticks):
        return (ticks / 4096) * 10
        
    def rps_to_enc_ticks_per_100_ms(self, rotations):
        return (rotations / 10) * 4096
 
    def __init__(self):
        super().__init__()

        self.angulatorMotor = VictorSPX(constants.kAngulatorMotorCANID)
        self.angulatorMotor.configFactoryDefault()
        self.angulatorMotor.setNeutralMode(NeutralMode.Brake)
        self.angulatorMotor.setInverted(InvertType.InvertMotorOutput)
        self.angulatorMotor.config_kP(0,constants.kAngulatorP)
        self.angulatorMotor.config_kI(0,constants.kAngulatorI)
        self.angulatorMotor.config_kD(0,constants.kAngulatorD)
        self.angulatorMotor.config_kF(0,constants.kAngulatorV)
        self.angulatorMotor.configMotionCruiseVelocity(self.rps_to_enc_ticks_per_100_ms(constants.kAngulatorCruiseVelocity))
        self.angulatorMotor.configMotionAcceleration(self.rps_to_enc_ticks_per_100_ms(constants.kAngulatorAcceleration))
        self.angulatorMotor.configMotionSCurveStrength(constants.kAngulatorJerk)
        self.angulatorMotor.configForwardSoftLimitThreshold(constants.kAngulatorForwardSoftLimit)
        self.angulatorMotor.configReverseSoftLimitThreshold(constants.kAngulatorReverseSoftLimit)
        self.angulatorMotor.configForwardSoftLimitEnable(True)
        self.angulatorMotor.configReverseSoftLimitEnable(True)
               

    def angulator_move_velocity_cmd (self, angulator_velocity):
       return runOnce(
           lambda:  self.angulatorMotor.set(ControlMode.Velocity(self.rps_to_enc_ticks_per_100_ms(angulator_velocity)))
        )
       
    def angulator_set_position_cmd (self, angulator_position):
           return runOnce(
           lambda:  self.angulatorMotor.set(ControlMode.MotionMagic.Position(self.rot_to_enc_ticks(angulator_position / 360.0)))
        )   

    def angulator_off_cmd (self):
        return runOnce(
           lambda: self.angulatorMotor.set(ControlMode.Disabled())
        )
    
    def anglator_set_position_from_range(self, range):
        self.angulatorMotor.set(ControlMode.MotionMagic.Position(self.rot_to_enc_ticks(math.atan(78/range))))

    def periodic(self):
        # Get the X and Y from the dashboard so we can set angulator to correct angle for this range.
        #pose_x = SmartDashboard.getNumber("X_POSE")
        #pose_y = SmartDashboard.getNumber("Y_POSE")
        
        SmartDashboard.putNumber("Angulator Position", self.enc_ticks_to_rot(self.angulatorMotor.getSelectedSensorPosition()))
        SmartDashboard.putNumber("Angulator Velocity", self. enc_ticks_per_100_ms_to_rps(self.angulatorMotor.getSelectedSensorVelocity()))
        SmartDashboard.putNumber("Angulator Voltage", self.angulatorMotor.getBusVoltage())
        SmartDashboard.putNumber("Angulator Active Trajectory Position", self.enc_ticks_to_rot(self.angulatorMotor.getActiveTrajectoryPosition()))
        SmartDashboard.putNumber("Angulator Active Trajectory Velocity", self.enc_ticks_per_100_ms_to_rps(self.angulatorMotor.getActiveTrajectoryVelocity()))

    


