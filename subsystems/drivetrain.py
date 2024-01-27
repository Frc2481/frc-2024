import math
import wpilib
import ntcore
import wpimath.units
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry, \
ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
#Pathplanner stuff
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpilib import DriverStation
from pathplannerlib.auto import PathPlannerAuto

from commands2.cmd import *

import commands2
from commands2.button import CommandXboxController 

from phoenix6.controls import VelocityVoltage, PositionVoltage

from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.cancoder_configs import CANcoderConfiguration 

from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware import TalonFX
from phoenix6.hardware import Pigeon2

import constants

import phoenix6
from phoenix6.signals.spn_enums import *


class SwerveModule(object):

    def __init__(self, driveCANID, steerCANID, steerCANCoderID):

        self.driveMotor = TalonFX(driveCANID)
        self.steerMotor = TalonFX(steerCANID)
        self.steerEncoder = CANcoder(steerCANCoderID)

        self.driveMotorConfig = TalonFXConfiguration()
        self.driveMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.driveMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.driveMotorConfig.slot0.k_p = constants.kdriveP
        self.driveMotorConfig.slot0.k_i = constants.kdriveI 
        self.driveMotorConfig.slot0.k_d = constants.kdriveD
        self.driveMotorConfig.slot0.k_v = constants.kdriveV 
        self.driveMotorConfig.feedback.sensor_to_mechanism_ratio = constants.kSwerveReductionDrive
        self.driveMotor.configurator.apply(self.driveMotorConfig)
        
        self.steerMotorConfig = TalonFXConfiguration()
        self.steerMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.steerMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.steerMotorConfig.slot0.k_p = constants.ksteerP
        self.steerMotorConfig.slot0.k_i = constants.ksteerI
        self.steerMotorConfig.slot0.k_d = constants.ksteerD
        self.steerMotorConfig.slot0.k_v = constants.ksteerV

        self.steerMotorConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        self.steerMotorConfig.feedback.feedback_remote_sensor_id = steerCANCoderID 
        self.steerMotorConfig.feedback.sensor_to_mechanism_ratio = 1.0
        self.steerMotorConfig.feedback.rotor_to_sensor_ratio = constants.kSwerveReductionSteer
        self.steerMotor.configurator.apply(self.steerMotorConfig)

        self.canCoderConfig = CANcoderConfiguration()
        self.canCoderConfig.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        self.canCoderConfig.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        self.canCoderConfig.magnet_sensor.magnet_offset = 0.4
        self.steerEncoder.configurator.apply(self.canCoderConfig)

        self.wheel_circumference = 0


    def zero_steer_encoder(self):
        #zeros the offset so we dont have to deal with the previous one
        self.canCoderConfig.magnet_sensor.magnet_offset = 0
        self.steerEncoder.configurator.apply(self.canCoderConfig)
        
        #Wait for fresh data after offset
        steer_offset = self.steerEncoder.get_absolute_position()
        steer_offset.wait_for_update(1)
        self.canCoderConfig.magnet_sensor.magnet_offset = -steer_offset.value
        self.steerEncoder.configurator.apply(self.canCoderConfig)
        return -steer_offset.value
    
    def set_steer_offset(self, steer_offset:float):
        self.canCoderConfig.magnet_sensor.magnet_offset = steer_offset
        self.steerEncoder.configurator.apply(self.canCoderConfig)
       
    def distance(self):
        return self.driveMotor.get_position().value * self.wheel_circumference
    
    
    def angle(self):
        return Rotation2d(self.steerEncoder.get_absolute_position().value * 2 * math.pi)
    
    def set_state(self, state):
        state = SwerveModuleState.optimize(state, self.angle())
        self.driveMotor.set_control(VelocityVoltage(state.speed))        
        self.steerMotor.set_control(PositionVoltage(state.angle.degrees() / 360))
    
    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            distance=self.distance(),
            angle=self.angle()
        )
   
    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            meters_per_second=wpimath.units.feetToMeters(self.driveMotor.get_velocity().value * (self.wheel_circumference)),
            angle=self.angle()
        )
    
    def zero_drive_encoder(self):
       self.driveMotor.set_position(0)
    
   

class DriveSubsystem(commands2.Subsystem):

    def __init__(self):
        super().__init__()

        self._fl = SwerveModule(constants.kSwerveFrontLeftDriveMotorCANID, 
                                 constants.kSwerveFrontLeftSteerMotorCANID, 
                                 constants.kSwerveFrontLeftSteerEncoderCANID)  
        self._fr = SwerveModule(constants.kSwerveFrontRightDriveMotorCANID, 
                                 constants.kSwerveFrontRightSteerMotorCANID,
                                 constants.kSwerveFrontRightSteerEncoderCANID)
        self._bl = SwerveModule(constants.kSwerveBackLeftDriveMotorCANID,
                                 constants.kSwerveBackLeftSteerMotorCANID, 
                                 constants.kSwerveBackLeftSteerEncoderCANID)
        self._br = SwerveModule(constants.kSwerveBackRightDriveMotorCANID,
                                 constants.kSwerveBackRightSteerMotorCANID, 
                                 constants.kSwerveBackrightSteerEncoderCANID) 
        self._fl.set_steer_offset(wpilib.Preferences.getDouble("FL_ENCODER_OFFSET"))
        self._fr.set_steer_offset(wpilib.Preferences.getDouble("FR_ENCODER_OFFSET"))
        self._bl.set_steer_offset(wpilib.Preferences.getDouble("BL_ENCODER_OFFSET"))
        self._br.set_steer_offset(wpilib.Preferences.getDouble("BR_ENCODER_OFFSET"))

        self._fl.wheel_circumference = wpilib.Preferences.getDouble("FL_WHEEL_CIRCUMFERENCE")
        self._fr.wheel_circumference = wpilib.Preferences.getDouble("FR_WHEEL_CIRCUMFERENCE")
        self._bl.wheel_circumference = wpilib.Preferences.getDouble("BL_WHEEL_CIRCUMFERENCE")
        self._br.wheel_circumference = wpilib.Preferences.getDouble("BR_WHEEL_CIRCUMFERENCE")

        self._gyro = Pigeon2(constants.kPigeonCANID)

        self.__kinematics = SwerveDrive4Kinematics(
            Translation2d(-constants.kWheelTrack / 2.0, constants.kWheelBase / 2.0),
            Translation2d(constants.kWheelTrack / 2.0, constants.kWheelBase / 2.0),
            Translation2d(-constants.kWheelTrack / 2.0, -constants.kWheelBase / 2.0),
            Translation2d(constants.kWheelTrack / 2.0, -constants.kWheelBase / 2.0),
        
                                                                                    
        )      
        self.__odometry = SwerveDrive4Odometry(
            kinematics = self.__kinematics,
            gyroAngle = Rotation2d(),
            modulePositions=(
                self._fl.get_position(),
                self._fr.get_position(),
                self._bl.get_position(),
                self._br.get_position()
            ),
            initialPose=Pose2d()
        )

        AutoBuilder.configureHolonomic(             
              self.get_pose,
              self.reset_pose,
              self.get_robot_relative_speed, 
              self.drive_robot_relative_speed, 
              HolonomicPathFollowerConfig( 
                  PIDConstants(5.0, 0.0, 0.0), 
                  PIDConstants(5.0, 0.0, 0.0),
                  constants.kMaxModuleSpeedFt / 3.281, 
                  constants.kDriveBaseRadiusIn / 39.37, 
                  ReplanningConfig() 
              ),
              self.shouldFlipPath, #starts on blue
              self 
        )
        
        self.__sd = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")
    
    def periodic(self):
        
        # FIXME: Crashes currently.
        self.__odometry.update(
            Rotation2d.fromDegrees(self._gyro.get_yaw().value),
             [
                self._fl.get_position(),
                self._fr.get_position(),
                self._bl.get_position(),
                self._br.get_position(),
             ]
        )
        
        self.__sd.putNumber("FLAngleActual", self._fl.get_position().angle.degrees())
        self.__sd.putNumber("FRAngleActual", self._fr.get_position().angle.degrees())
        self.__sd.putNumber("BLAngleActual", self._bl.get_position().angle.degrees())
        self.__sd.putNumber("BRAngleActual", self._br.get_position().angle.degrees())
        self.__sd.putNumber("GYROFINISHED", self._gyro.get_yaw().value)
        self.__sd.putNumber("FL_DISTANCE_ACTUAL",self._fl.get_position().distance)
        self.__sd.putNumber("FR_DISTANCE_ACTUAL",self._fr.get_position().distance)
        self.__sd.putNumber("BL_DISTANCE_ACTUAL",self._bl.get_position().distance)
        self.__sd.putNumber("BR_DISTANCE_ACTUAL",self._br.get_position().distance)
    

    def get_pose(self):
        return self.__odometry.getPose()
    
    def reset_pose(self, pose):
            self.__odometry.resetPosition(
                self._gyro.get_yaw().value,
                self._fl.get_position(),
                self._fr.get_position(),
                self._bl.get_position(),
                self._br.get_position(),
                pose
            )
    def drive(self, x, y, theta, field_relative, force_angle=False):
        if field_relative:
            chassis_speed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, self._gyro.get_yaw().value)
        else:
            chassis_speed = ChassisSpeeds(x, y, theta)

        self.drive_robot_relative_speed(chassis_speed, force_angle)

    def drive_robot_relative_speed(self, chassis_speed, force_angle):

        chassis_speed = ChassisSpeeds.discretize(chassis_speed, constants.kDrivePeriod)

        module_states = self.__kinematics.toSwerveModuleStates(chassis_speed)

        module_states = self.__kinematics.desaturateWheelSpeeds(module_states, constants.kDriveMaxSpeed)

        if force_angle:
            module_states[0].speed = 0
            module_states[1].speed = 0
            module_states[2].speed = 0
            module_states[3].speed = 0

        self._fl.set_state(module_states[0])
        self._fr.set_state(module_states[1])
        self._bl.set_state(module_states[2])
        self._br.set_state(module_states[3])

        self.__sd.putNumber("FLAngle", module_states[0].angle.degrees())
        self.__sd.putNumber("FRAngle", module_states[1].angle.degrees())
        self.__sd.putNumber("BLAngle", module_states[2].angle.degrees())
        self.__sd.putNumber("BRAngle", module_states[3].angle.degrees())        
        self.__sd.putNumber("FLSpeed", module_states[0].speed)
        self.__sd.putNumber("FRSpeed", module_states[1].speed)
        self.__sd.putNumber("BLSpeed", module_states[2].speed)
        self.__sd.putNumber("BRSpeed", module_states[3].speed)

        

    def get_robot_relative_speed(self):
        return self.__kinematics.toChassisSpeeds(
            (
                self._fl.get_state(),
                self._fr.get_state(),
                self._bl.get_state(),
                self._br.get_state() 
            )
        )
    
    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def drive_with_joystick_cmd(self, joystick: CommandXboxController):
        return runEnd(
            lambda: self.drive(joystick.getLeftX(),
                               joystick.getLeftY(),
                               joystick.getRightX(),
                               False
                                ),
            lambda: self.drive(0, 0, 0, False), 
            self
        )
    def zero_drive_encoder(self):
        self._fl.zero_drive_encoder()
        self._fr.zero_drive_encoder()
        self._bl.zero_drive_encoder()
        self._br.zero_drive_encoder()

    def zero_steer_encoder(self):
        wpilib.Preferences.setDouble("FL_STEER_OFFSET", self._fl.zero_steer_encoder())
        wpilib.Preferences.setDouble("FR_STEER_OFFSET", self._fr.zero_steer_encoder())
        wpilib.Preferences.setDouble("BL_STEER_OFFSET", self._bl.zero_steer_encoder())
        wpilib.Preferences.setDouble("BR_STEER_OFFSET", self._br.zero_steer_encoder())
        
    def zero_steer_encoder_cmd(self):

                    return runOnce(self.zero_steer_encoder)
    
    def finalize_calibrate_wheel_circumfrence(self):
        distance_traveled_in = constants.kDriveBaseRadiusIn * math.radians(self._gyro.get_yaw().value + 360)
        self._fl.wheel_circumference = distance_traveled_in / self._fl.driveMotor.get_rotor_position().value
        self._fr.wheel_circumference = distance_traveled_in / self._fr.driveMotor.get_rotor_position().value
        self._bl.wheel_circumference = distance_traveled_in / self._bl.driveMotor.get_rotor_position().value
        self._br.wheel_circumference = distance_traveled_in / self._br.driveMotor.get_rotor_position().value
        wpilib.Preferences.setDouble("FL_WHEEL_CIRCUMFERENCE", self._fl.wheel_circumference)
        wpilib.Preferences.setDouble("FR_WHEEL_CIRCUMFERENCE", self._fr.wheel_circumference)
        wpilib.Preferences.setDouble("BL_WHEEL_CIRCUMFERENCE", self._bl.wheel_circumference)
        wpilib.Preferences.setDouble("BR_WHEEL_CIRCUMFERENCE", self._br.wheel_circumference)


         
         
    def calibrate_wheel_circumference_cmd(self):
        return sequence(
            # Keep track of the gyro angle at the beginning of the cal process.
            InstantCommand(
                lambda: self._gyro.set_yaw(0)),
            
            # Point all wheels at a ~45 deg.
            InstantCommand(
                lambda: self.drive(0.0, 0.0, theta=1.0, field_relative=False,force_angle=True)), 
            WaitCommand(1),

            # Keep track of the wheel distance at the beginning of the cal process.
            InstantCommand(self.zero_drive_encoder),
            PrintCommand ("Zero Drive Encoder"),

            FunctionalCommand(
                lambda: None,
                lambda: self.drive(0.0, 0.0, theta=0.2, field_relative=False),
                lambda interupted: self.drive(0.0, 0.0, 0.0, field_relative=False),
                lambda: self._gyro.get_yaw().value + 360 if self._gyro.get_yaw().value else self._gyro.get_yaw().value> 358 * 1
            ),
            PrintCommand("Yaw Finished"),
            
            InstantCommand(self.finalize_calibrate_wheel_circumfrence)
        )
        
        
        #zero drive wheels
        #point wheels to 45 degrees
        #spin X number of times
        #figure distance travled woth gyroscope
        #figure each individual wheels rotations