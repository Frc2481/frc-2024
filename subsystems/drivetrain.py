import math
import json
import wpilib
import ntcore
import wpimath.units
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry, \
ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d, Pose2d, Twist2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import PIDController
#Pathplanner stuff
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpilib import DriverStation

from commands2 import WaitCommand, InstantCommand, FunctionalCommand, PrintCommand
from commands2 import *
from commands2 import sysid
from commands2.cmd import *

from commands2.button import CommandXboxController 

from ntcore import NetworkTableInstance, NetworkTable, NetworkTableEntry

from phoenix6.controls import VelocityVoltage, MotionMagicVoltage, VoltageOut, DutyCycleOut

from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.cancoder_configs import CANcoderConfiguration 

from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware import TalonFX
from phoenix6.hardware import Pigeon2
from phoenix6.status_signal import BaseStatusSignal

import constants

import phoenix6
from phoenix6.signals.spn_enums import *


from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController

from wpilib.sysid import SysIdRoutineLog
from wpiutil.log import StringLogEntry
from wpilib import DataLogManager
from wpilib import SmartDashboard, Field2d

from utils import *

from subsystems.swervemodule import SwerveModule


class DriveSubsystem(Subsystem):

    def __init__(self):
        super().__init__()

        self._fl = SwerveModule(constants.kSwerveFrontLeftDriveMotorCANID, 
                                 constants.kSwerveFrontLeftSteerMotorCANID, 
                                 constants.kSwerveFrontLeftSteerEncoderCANID,
                                 False)  
        self._fr = SwerveModule(constants.kSwerveFrontRightDriveMotorCANID, 
                                 constants.kSwerveFrontRightSteerMotorCANID,
                                 constants.kSwerveFrontRightSteerEncoderCANID,
                                 False)
        self._bl = SwerveModule(constants.kSwerveBackLeftDriveMotorCANID,
                                 constants.kSwerveBackLeftSteerMotorCANID, 
                                 constants.kSwerveBackLeftSteerEncoderCANID,
                                 True)
        self._br = SwerveModule(constants.kSwerveBackRightDriveMotorCANID,
                                 constants.kSwerveBackRightSteerMotorCANID, 
                                 constants.kSwerveBackrightSteerEncoderCANID,
                                 False) 
        self._fl.set_steer_offset(wpilib.Preferences.getDouble("FL_STEER_OFFSET"))
        self._fr.set_steer_offset(wpilib.Preferences.getDouble("FR_STEER_OFFSET"))
        self._bl.set_steer_offset(wpilib.Preferences.getDouble("BL_STEER_OFFSET"))
        self._br.set_steer_offset(wpilib.Preferences.getDouble("BR_STEER_OFFSET"))

        self._fl.wheel_circumference = abs(wpilib.Preferences.getDouble("FL_WHEEL_CIRCUMFERENCE", 9.425))
        self._fr.wheel_circumference = abs(wpilib.Preferences.getDouble("FR_WHEEL_CIRCUMFERENCE", 9.425))
        self._bl.wheel_circumference = abs(wpilib.Preferences.getDouble("BL_WHEEL_CIRCUMFERENCE", 9.425))
        self._br.wheel_circumference = abs(wpilib.Preferences.getDouble("BR_WHEEL_CIRCUMFERENCE", 9.425))
        
        self.yaw_pid = PIDController(
            Kp=wpilib.Preferences.getDouble("SPEAKER_YAW_P", 0.1),
            Ki=wpilib.Preferences.getDouble("SPEAKER_YAW_I", 0.0),
            Kd=wpilib.Preferences.getDouble("SPEAKER_YAW_D", 0.0)
        )
        self.yaw_pid.enableContinuousInput(0, 360)
        
        self.field = Field2d()
        
        SmartDashboard.putData("Field", self.field) 

        self._gyro = Pigeon2(constants.kPigeonCANID, "2481")
        
        
        self.__kinematics = SwerveDrive4Kinematics(
            Translation2d(constants.kWheelBase / 2.0, constants.kWheelTrack / 2.0),
            Translation2d(constants.kWheelBase / 2.0, -constants.kWheelTrack / 2.0),
            Translation2d(-constants.kWheelBase / 2.0, constants.kWheelTrack / 2.0),
            Translation2d(-constants.kWheelBase / 2.0, -constants.kWheelTrack / 2.0))

        self.__odometry = SwerveDrive4PoseEstimator(
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
        
        self.robot_relative_driving = False

        SmartDashboard.putNumber("limelight lateral gain", 1.0)
        SmartDashboard.putNumber("limelight angle gain", 0.1)
        
        if wpilib.Preferences.getDouble("NOTE_LATERAL_GAIN", 0.0) == 0:
            wpilib.Preferences.setDouble("NOTE_LATERAL_GAIN", 0.1)
                    
        AutoBuilder.configureHolonomic(
            self.get_pose, # Robot pose supplier
            self.reset_pose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.get_robot_relative_speed, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.drive_robot_relative_speed, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(2.2, 0.0, 0.1), # Rotation PID constants
                7.01, # Max module speed, in m/s
                0.45, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig(enableInitialReplanning=False) # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )
        PPHolonomicDriveController.setRotationTargetOverride(self.getRotationTargetOverride)
        
          
        self.__loggerState = None
        
        self.__sysid_config = sysid.SysIdRoutine.Config(recordState=self.logState)
        self.__sysid_mechanism = sysid.SysIdRoutine.Mechanism(self.driveVoltage, self.driveLog, self, "drive")
        
        self.__sysid = sysid.SysIdRoutine(self.__sysid_config, self.__sysid_mechanism)
        
        #self.ll_rear_table = NetworkTableInstance.getDefault().getTable("limelight-rear")
        self.ll_top_front_entry = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose_wpiblue")
        self.ll_top_back_entry = NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("botpose_wpiblue")
        self.ll_top_left_entry = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("botpose_wpiblue")
        self.ll_top_right_entry = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("botpose_wpiblue")
        self.ll_note_table = NetworkTableInstance.getDefault().getTable("limelight-note")
    
        self.drive_state = True
        self.note_correction = False
        self.auto_face_goal = False
        self.__cached_range_to_speaker = 0
        self.speaker_end_time_prev = 0
        self.note_correction_gain = 0
        
        SmartDashboard.putNumber("Angle Override", 0)
        
    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def getRotationTargetOverride(self):
        if self.auto_face_goal:
            return self.get_path_override_angle_to_speaker()
        return None
    
    def set_auto_face_goal(self, enabled):
        self.auto_face_goal = enabled
        
    def set_auto_face_goal_cmd(self, enabled):
        return InstantCommand(lambda: self.set_auto_face_goal(enabled))
    
    def periodic(self):

        start_time = wpilib.Timer.getFPGATimestamp()   
        
        BaseStatusSignal.refresh_all(self._fl.allSignals + self._fr.allSignals + self._bl.allSignals + self._br.allSignals)  

        self._fl.update()
        self._fr.update()
        self._bl.update()
        self._br.update()

        self.__odometry.update(
            Rotation2d.fromDegrees(self._gyro.get_yaw().value),
             [
                self._fl.get_position(),
                self._fr.get_position(),
                self._bl.get_position(),
                self._br.get_position(),
             ]
        )
        self.limelight_periodic()
        self.dashboard_periodic()
        
        self.get_range_to_speaker(cached=False)
        
        end_time = wpilib.Timer.getFPGATimestamp()
        SmartDashboard.putNumber("drive_loop", end_time - start_time)
        
    def add_pose_from_limelight(self, nt_entry : NetworkTableEntry, ll_name):
        
        bot_pose = nt_entry.getDoubleArray([0,0,0,0,0,0,0,0,0,0])
        
        if bot_pose[7] > 0:
            # 0.tx
            # 1.ty,
            # 2.tz
            # 3.roll
            # 4.pitch
            # 5.yaw
            # 6.latency
            # 7.tag count
            # 8. tag span meters
            # 9. average distance meters
            # 10. average area % of image
            # 11 + n*7, tag id
            # 12 + n*7 tx nc
            # 13 + n*7 ty nc
            # 14 + n*7 ta
            # 15 + n*7 distance to camera
            # 16 + n*7 distance to robot
            # 17 + n*7 ambiguity
            
            
            num_targets = bot_pose[7]
            # total_latency_ms = nt_table.getNumber("cl",0) + \
            #                    nt_table.getNumber("tl",0)
            total_latency_ms = bot_pose[6] 
            capture_timestamp_sec = wpilib.Timer.getFPGATimestamp() - total_latency_ms / 1000.0
            vision_pose = Pose2d(x=bot_pose[0],
                                 y=bot_pose[1],
                                 rotation=Rotation2d.fromDegrees(bot_pose[5]))
            # try:
            single_tag_ambiguity = bot_pose[17] if len(bot_pose) >= 18 else 0
            single_tag_distance = bot_pose[15] if len(bot_pose) >= 16 else 0
            # except Exception as e:
                # print(e)
            bad_pose = Pose2d(-100, -100, Rotation2d())

            if (vision_pose.X() == 0.0): 
                self.field.getObject(ll_name).setPose(bad_pose) 
                return 

            if num_targets >= 2:
                self.__odometry.setVisionMeasurementStdDevs((0.7, 0.7, 99999999))
                self.__odometry.addVisionMeasurement(vision_pose, capture_timestamp_sec)
                self.field.getObject(ll_name).setPose(vision_pose)
                
            elif num_targets == 1 and single_tag_ambiguity < 0.3 and single_tag_distance <= 5:
                self.__odometry.setVisionMeasurementStdDevs((2.0, 2.0, 99999999))
                self.__odometry.addVisionMeasurement(vision_pose, capture_timestamp_sec)
                self.field.getObject(ll_name).setPose(vision_pose)
            else: 
                self.field.getObject(ll_name).setPose(bad_pose)
    
    # Lime Light Update 
    def limelight_periodic(self):       
        #checks if april tag is visible
        self.add_pose_from_limelight(self.ll_top_front_entry, "ll_top_front")
        self.add_pose_from_limelight(self.ll_top_back_entry, "ll_top_back")
        self.add_pose_from_limelight(self.ll_top_left_entry, "ll_top_left")
        self.add_pose_from_limelight(self.ll_top_right_entry, "ll_top_right")
        #self.add_pose_from_limelight(self.ll_rear_table, "ll_rear")
        
     
    def dashboard_periodic(self):                                                       
        # SmartDashboard.putNumber("FL_Angle_Actual", self._fl.get_position().angle.degrees())
        # SmartDashboard.putNumber("FL_Distance",self._fl.get_position().distance)
        #SmartDashboard.putNumber("FL_Velocity",self._fl.driveMotor.get_rotor_velocity().value)
        # SmartDashboard.putNumber("FL_Voltage",self._fl.get_voltage())
        # SmartDashboard.putNumber("FL Duty Cycle", self._fl.driveMotor.get_duty_cycle().value)
        #SmartDashboard.putNumber("FL Current", self._fl.driveMotor.get_supply_current().value)        
        
        # SmartDashboard.putNumber("FR_Angle_Actual", self._fr.get_position().angle.degrees())
        # SmartDashboard.putNumber("FR_Distance",self._fr.get_position().distance)
        #SmartDashboard.putNumber("FR_Velocity",self._fr.driveMotor.get_rotor_velocity().value)
        # SmartDashboard.putNumber("FR_Voltage",self._fr.get_voltage())
        # SmartDashboard.putNumber("FR Duty Cycle", self._fr.driveMotor.get_duty_cycle().value)
        #SmartDashboard.putNumber("FR Current", self._fr.driveMotor.get_supply_current().value)
        
        # SmartDashboard.putNumber("BL_Angle_Actual", self._bl.get_position().angle.degrees())
        # SmartDashboard.putNumber("BL_Distance",self._bl.get_position().distance)
        #SmartDashboard.putNumber("BL_Velocity",self._bl.driveMotor.get_rotor_velocity().value)
        #SmartDashboard.putNumber("BL_Voltage",self._bl.get_voltage())
        #SmartDashboard.putNumber("BL Duty Cycle", self._bl.driveMotor.get_duty_cycle().value)
        #SmartDashboard.putNumber("BL Current", self._bl.driveMotor.get_supply_current().value)
        
        
        # SmartDashboard.putNumber("BR_Angle_Actual", self._br.get_position().angle.degrees())      
        # SmartDashboard.putNumber("BR_Distance",self._br.get_position().distance)
        #SmartDashboard.putNumber("BR_Velocity",self._br.driveMotor.get_rotor_velocity().value)
        #SmartDashboard.putNumber("BR_Voltage",self._br.get_voltage())
        #SmartDashboard.putNumber("BR Duty Cycle", self._br.driveMotor.get_duty_cycle().value)
        #SmartDashboard.putNumber("BR Current", self._fr.driveMotor.get_supply_current().value)
        
        #SmartDashboard.putNumber("BR Supply Voltage", self._br.driveMotor.get_supply_voltage().value)
        #SmartDashboard.putNumber("FR Supply Voltage", self._fr.driveMotor.get_supply_voltage().value)
        #SmartDashboard.putNumber("FL Supply Voltage", self._fl.driveMotor.get_supply_voltage().value)
        #SmartDashboard.putNumber("BL Supply Voltage", self._bl.driveMotor.get_supply_voltage().value)
        
        # SmartDashboard.putNumber("Yaw", self._gyro.get_yaw().value)
        # SmartDashboard.putNumber("X_POSE", self.get_pose().x)
        # SmartDashboard.putNumber("Y_POSE", self.get_pose().y)
        
        #SmartDashboard.putNumber("Angle to Speaker", self.get_angle_to_speaker())
        #SmartDashboard.putNumber("Distance to Speaker", self.get_range_to_speaker())

        #SmartDashboard.putNumber("FR Steer Velocity",self._fr.steerMotor.get_rotor_velocity())
        #SmartDashboard.putNumber("Bl Steer Velocity",self._bl.steerMotor.get_rotor_velocity())
        #SmartDashboard.putNumber("BR Steer Velocity",self._br.steerMotor.get_rotor_velocity())
        #SmartDashboard.putNumber("FL Steer Velocity",self._fl.steerMotor.get_rotor_velocity())
                    
        self.field.setRobotPose(self.__odometry.getEstimatedPosition())   
        
        if DriverStation.isDisabled():
            self.yaw_pid.setPID(
                wpilib.Preferences.getDouble("SPEAKER_YAW_P", 0.0),
                wpilib.Preferences.getDouble("SPEAKER_YAW_I", 0.0),
                wpilib.Preferences.getDouble("SPEAKER_YAW_D", 0.0),
            )
            
            self.note_correction_gain = wpilib.Preferences.getDouble("NOTE_LATERAL_GAIN", 0.1)
        
    # Odometry   
    def get_pose(self, delta=0.0, is_blue=True) -> Pose2d:        
        if delta == 0.0:
            return self.__odometry.getEstimatedPosition()
        
        cs = self.get_robot_relative_speed()
        cs = cs.fromRobotRelativeSpeeds(cs, Rotation2d.fromDegrees(self._gyro.get_yaw().value))
        if is_blue:
            twist = Twist2d(cs.vx, cs.vy, cs.omega) # meters per second
        else:
            twist = Twist2d(-cs.vx, -cs.vy, cs.omega) # meters per second
        twist *= delta
        pose = self.__odometry.getEstimatedPosition()
        pose = pose.exp(twist)
        return pose

    def reset_pose(self, pose=Pose2d()):         
        self._gyro.set_yaw(pose.rotation().degrees())
            
        self.__odometry.resetPosition(    
            pose.rotation(),
            (
                self._fl.get_position(),
                self._fr.get_position(),
                self._bl.get_position(),
                self._br.get_position()
            ),
            pose
        )
   
    def reset_yaw(self):
        if self.shouldFlipPath():
            self.reset_pose(self.get_pose()
                            .rotateBy(self.get_pose().rotation() * -1) # Subtract out current angle of robot
                            .rotateBy(Rotation2d.fromDegrees(180))) # Rotate 180 for the red alliance
        else:
            self.reset_pose(self.get_pose()
                            .rotateBy(self.get_pose().rotation() * -1)) # Subtract out current angle of robot
        
    def reset_yaw_cmd(self):
        return runOnce(lambda: self.reset_yaw())
            
    def reset_odom_to_vision(self):
        bot_pose = self.ll_rear_table.getEntry("botpose.wpiblue").getDoubleArray([0,0,0,0,0,0])

        vision_pose = Pose2d(x=bot_pose[0],
                             y=bot_pose[1],
                             rotation=Rotation2d.fromDegrees(bot_pose[3]))
        self.reset_pose(vision_pose)
    
    def reset_odom_to_vision_cmd(self):
        return runOnce (self.reset_odom_to_vision)
    
    def get_range_to_speaker(self, cached=True):
        if cached:
            return self.__cached_range_to_speaker
    
        else:
            start_time = wpilib.Timer.getFPGATimestamp()
            
            look_ahead_time = 0.2 # wpilib.Preferences.getDouble("LOOK_AHEAD_TIME", 0.1)
            if self.shouldFlipPath():
                self.__cached_range_to_speaker = self.get_pose(look_ahead_time, is_blue=False).relativeTo(constants.kRedSpeakerPose).translation().norm() 
                #5.547 original  
            else:
                self.__cached_range_to_speaker = self.get_pose(look_ahead_time, is_blue=True).relativeTo(constants.kBlueSpeakerPose).translation().norm()   
            
            end_time = wpilib.Timer.getFPGATimestamp()
            dt = end_time - start_time
            if dt < 0.2:
                SmartDashboard.putNumber("range_loop", dt)
            
            return self.__cached_range_to_speaker
    
    # Drive Controls
                
    def drive(self, x, y, theta, field_relative, force_angle=False):
                   
        if field_relative:
            if self.shouldFlipPath():
                x *= -1
                y *= -1    
            
            #yaw_comped = self._gyro.get_yaw().value + (self._gyro.get_angular_velocity_z_world().value / 5.0)
            chassis_speed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, Rotation2d.fromDegrees(self._gyro.get_yaw().value)) #  self._gyro.get_yaw().value
        else:
            chassis_speed = ChassisSpeeds(x, y, theta)

        self.drive_robot_relative_speed(chassis_speed, force_angle, True)
        

    def toggle_robot_relative_driving(self):
        self.robot_relative_driving = not self.robot_relative_driving
        
    def toggle_robot_relative_driving_cmd(self):
        return runOnce(self.toggle_robot_relative_driving)
    
    def set_field_centric(self, field_centric):
        self.drive_state = field_centric
    
    def field_centric_cmd(self):
        return runOnce(lambda: self.set_field_centric(True))
        
    def robot_centric_cmd(self):
        return runOnce(lambda: self.set_field_centric(False))
    
    def set_correct_path_to_note(self, note_correction):
        self.note_correction = note_correction
        self.has_seen_note = False
    
    def set_correct_path_to_note_cmd(self, enabled):
        return runOnce(lambda: self.set_correct_path_to_note(enabled))
    
    def get_note_correction_speed(self):
        if self.is_note_visible():
            self.has_seen_note = True
            return -self.ll_note_table.getNumber('tx', 0) * self.note_correction_gain
        else:
            return 0.0
    
    def is_note_visible(self):
        return self.ll_note_table.getNumber('tv', 0)
          
    def drive_robot_relative_speed(self, chassis_speed: ChassisSpeeds, force_angle=False, voltage_only=False):
        
        # SmartDashboard.putNumber("Target Omega", chassis_speed.omega)

    
        #chassis_speed = ChassisSpeeds.discretize(chassis_speed, constants.kDrivePeriod) #-Should be using
        
        # Only override the y velocity vector when note correction is avtive and we see a note.
        note_correction_speed = 0.0 
        if self.note_correction:
            note_correction_speed = self.get_note_correction_speed()
            if self.has_seen_note and abs(chassis_speed.vx) > 0.1:
                chassis_speed.vy = note_correction_speed
            
        # SmartDashboard.putNumber("Note Correction Speed", note_correction_speed)
        
        # SmartDashboard.putNumber("Chassis Speed X", chassis_speed.vx)
        # SmartDashboard.putNumber("Chassis Speed Y", chassis_speed.vy)

        module_states = self.__kinematics.toSwerveModuleStates(chassis_speed)

        module_states = self.__kinematics.desaturateWheelSpeeds(module_states, constants.kDriveMaxSpeed)

        if force_angle:
            module_states[0].speed = 0
            module_states[1].speed = 0
            module_states[2].speed = 0
            module_states[3].speed = 0

        self._fl.set_state(module_states[0], voltage_only)
        self._fr.set_state(module_states[1], voltage_only)
        self._bl.set_state(module_states[2], voltage_only)
        self._br.set_state(module_states[3], voltage_only)

    def get_robot_relative_speed(self):
        cs = self.__kinematics.toChassisSpeeds(
            (
                self._fl.get_state(),
                self._fr.get_state(),
                self._bl.get_state(),
                self._br.get_state() 
            )
        )
        
        # SmartDashboard.putNumber("Actual Omega", cs.omega)
        # SmartDashboard.putNumber("Actual Vx", cs.vx)
        # SmartDashboard.putNumber("Actual Vy", cs.vy)
        
        return cs
    
    def drive_with_joystick_cmd(self, joystick: CommandXboxController):
        return runEnd(
            lambda: self.drive(scale_axis(-joystick.getLeftY()) * constants.kDriveMaxSpeed,
                            scale_axis(-joystick.getLeftX()) * constants.kDriveMaxSpeed,
                            scale_axis(-joystick.getRightX()) * 6,
                            self.drive_state
                            ),
            lambda: None, # self.drive(0, 0, 0, True) 
            self
        )
    
    def limelight_note_align_cmd(self, joystick: CommandXboxController):
        return RepeatCommand(
                SequentialCommandGroup(
                    self.drive_with_joystick_cmd(joystick).raceWith(self.wait_for_note_visible()),
                    self.drive_towards_note_command(joystick).raceWith(self.wait_for_no_note_visible())          
            )   
        )
        
    def limelight_speaker_align_cmd(self, joystick: CommandXboxController):
        return self.drive_speaker_aligned_cmd(joystick)        

    def limelight_amp_align_cmd(self, joystick: CommandXboxController):
        return self.amp_lineup_cmd(joystick)
        
    def limelight_align_cmd(self, joystick: CommandXboxController, state_cb):
        return SelectCommand({
            constants.kAlignStateAmp: self.limelight_amp_align_cmd(joystick),
            constants.kAlignStateNote: self.limelight_note_align_cmd(joystick),
            constants.kAlignStateSpeaker: self.limelight_speaker_align_cmd(joystick)
        },
        state_cb)
        
    def wait_for_note_visible(self):
        return WaitUntilCommand(lambda: NetworkTableInstance.getDefault().getTable("limelight-front").getNumber('tv', 0) == 1)
    
    def wait_for_no_note_visible(self):
        return WaitUntilCommand(lambda: NetworkTableInstance.getDefault().getTable("limelight-front").getNumber('tv', 0) == 0)
        
    def drive_towards_note_command(self, joystick: CommandXboxController):
        return runEnd( 
            lambda: self.drive(scale_axis(math.hypot(joystick.getLeftY(), joystick.getLeftX())) * constants.kDriveMaxSpeed,
                                0,
                                scale_axis(-NetworkTableInstance.getDefault().getTable("limelight-front").getNumber('tx', 0) *
                                SmartDashboard.getNumber("limelight angle gain", 0)),
                                False
                    ), 
            lambda: None,
            self
        )
    
    def get_path_override_angle_to_speaker(self):
            # TODO: Possible add a trim if this isn't perfect.
        if self.shouldFlipPath():
            translation_to_speaker = self.get_pose(0.3).relativeTo(constants.kRedSpeakerPose).translation()   
        else:
            translation_to_speaker = self.get_pose(0.3).relativeTo(constants.kBlueSpeakerPose).translation()
        
        # SmartDashboard.putNumber("Angle Override", translation_to_speaker.angle().degrees())
        
        # translation_to_speaker = self.get_pose().relativeTo(Pose2d(-0.0381, 5.547, Rotation2d())).translation()
        return translation_to_speaker.angle()
    
    def get_angle_to_speaker(self):
        # TODO: Possible add a trim if this isn't perfect.
        if self.shouldFlipPath():
            translation_to_speaker = self.get_pose(0.3, is_blue=False).relativeTo(constants.kRedSpeakerPose).translation()   
        else:
            translation_to_speaker = self.get_pose(0.3, is_blue=True).relativeTo(constants.kBlueSpeakerPose).translation()
        
        # translation_to_speaker = self.get_pose().relativeTo(Pose2d(-0.0381, 5.547, Rotation2d())).translation()
        # SmartDashboard.putNumber("Speaker Target Angle", translation_to_speaker.angle().degrees())
        angle_to_speaker = translation_to_speaker.angle().rotateBy(-self.get_pose().rotation())
        SmartDashboard.putNumber("Speaker Angle Error", angle_to_speaker.degrees())
        return translation_to_speaker.angle().degrees()
        
    def get_omega_from_angle_to_speaker(self):    
        
        end_time = wpilib.Timer.getFPGATimestamp()
        dt = end_time - self.speaker_end_time_prev
        if dt < 0.2:
            SmartDashboard.putNumber("speaker_loop", dt)
        self.speaker_end_time_prev = end_time
        
        self.yaw_pid.setSetpoint(self.get_angle_to_speaker())
        pid_output = self.yaw_pid.calculate(self.get_pose().rotation().degrees())
        cs = self.get_robot_relative_speed()
        speed = (cs.vx ** 2 + cs.vy ** 2) ** 0.5
        return pid_output
        return ((0.5 * pid_output) *(1 - (speed / constants.kDriveMaxSpeed))) + (pid_output * 0.5)
        
    
    def drive_speaker_aligned_cmd(self, joystick: CommandXboxController):
        return runEnd( 
            lambda: self.drive(scale_axis(-joystick.getLeftY()) * constants.kDriveMaxSpeed,
                            scale_axis(-joystick.getLeftX()) * constants.kDriveMaxSpeed,
                                self.get_omega_from_angle_to_speaker(),
                                # self.get_angle_to_speaker() * SmartDashboard.getNumber("limelight angle gain", 0),
                                #-NetworkTableInstance.getDefault().getTable("limelight-rear").getNumber('tx', 0) *
                                #SmartDashboard.getNumber("limelight gain", 0),
                                True
                    ), 
            lambda: None,
            self
        )
    
    def get_lateral_distance_amp(self):
        # TODO: TODO: TODO: TODO: TODO:
        # TODO: Make this work for red alliance too.
        # TODO: Possible add a trim if this isn't perfect.
        # FIXME: Why do we have to fudge this 20cm?
        translation_to_amp = self.get_pose().relativeTo(Pose2d(2.0415, 8.2042, Rotation2d.fromDegrees(270))).translation()
        return -translation_to_amp.Y()
        angle_to_speaker = translation_to_speaker.angle().rotateBy(-self.get_pose().rotation())
        return angle_to_speaker.degrees()
    
    def get_angle_to_amp(self):
        # TODO: Make this work for red alliance too.
        return Rotation2d.fromDegrees(270).rotateBy(-self.get_pose().rotation()).degrees()
    
    def amp_lineup_cmd(self, joystick: CommandXboxController):
        return runEnd(
            lambda: self.drive(self.get_lateral_distance_amp() * SmartDashboard.getNumber("limelight lateral gain", 0),
                                scale_axis(-joystick.getLeftY() * constants.kDriveMaxSpeed),                               
                               self.get_angle_to_amp() * SmartDashboard.getNumber("limelight angle gain", 0),
                               True
                    ),
            lambda: None,
            self
        )
        
    def wait_for_target_visible(self):
        return WaitUntilCommand(lambda: NetworkTableInstance.getDefault().getTable("limelight-rear").getNumber('tv', 0) == 1)
    
    def wait_for_no_target_visible(self):
        return WaitUntilCommand(lambda: NetworkTableInstance.getDefault().getTable("limelight-rear").getNumber('tv', 0) == 0)
                
    # Zero Drive Encoder
    
    def zero_drive_encoder(self):
        self._fl.zero_drive_encoder()
        self._fr.zero_drive_encoder()
        self._bl.zero_drive_encoder()
        self._br.zero_drive_encoder()
                
    def zero_drive_encoder_cmd(self):
        return runOnce(self.zero_drive_encoder).ignoringDisable(True)

    # Zero Steer Encoder

    def zero_steer_encoder(self):
        wpilib.Preferences.setDouble("FL_STEER_OFFSET", self._fl.zero_steer_encoder())
        wpilib.Preferences.setDouble("FR_STEER_OFFSET", self._fr.zero_steer_encoder())
        wpilib.Preferences.setDouble("BL_STEER_OFFSET", self._bl.zero_steer_encoder())
        wpilib.Preferences.setDouble("BR_STEER_OFFSET", self._br.zero_steer_encoder())
        
    def zero_steer_encoder_cmd(self):
        return runOnce(self.zero_steer_encoder).ignoringDisable(True)

    # Wheel Circumference
    
    def finalize_calibrate_wheel_circumfrence(self):
        distance_traveled_in = constants.kDriveBaseRadiusIn * math.radians(self._gyro.get_yaw().value)
        self._fl.wheel_circumference = distance_traveled_in / self._fl.driveMotor.get_position().value
        self._fr.wheel_circumference = distance_traveled_in / self._fr.driveMotor.get_position().value
        self._bl.wheel_circumference = distance_traveled_in / self._bl.driveMotor.get_position().value
        self._br.wheel_circumference = distance_traveled_in / self._br.driveMotor.get_position().value
        wpilib.Preferences.setDouble("FL_WHEEL_CIRCUMFERENCE", self._fl.wheel_circumference)
        wpilib.Preferences.setDouble("FR_WHEEL_CIRCUMFERENCE", self._fr.wheel_circumference)
        wpilib.Preferences.setDouble("BL_WHEEL_CIRCUMFERENCE", self._bl.wheel_circumference)
        wpilib.Preferences.setDouble("BR_WHEEL_CIRCUMFERENCE", self._br.wheel_circumference)         
         
    def calibrate_wheel_circumference_cmd(self):
        return sequence(
            # Keep track of the gyro angle at the beginning of the cal process.
            InstantCommand(
                lambda: self._gyro.set_yaw(0)),
            PrintCommand("Set Yaw"),
            # Point all wheels at a ~45 deg.
            InstantCommand(
                lambda: self.drive(0.0, 0.0, theta=2.0, field_relative=False, force_angle=True)), 
            WaitCommand(1),
            PrintCommand("Wheels to a 45"),

            # Keep track of the wheel distance at the beginning of the cal process.
            InstantCommand(self.zero_drive_encoder),
            PrintCommand ("Zero Drive Encoder"),

            FunctionalCommand(
                lambda: None,
                lambda: self.drive(0.0, 0.0, theta=2, field_relative=False),
                lambda interupted: self.drive(0.0, 0.0, 0.0, field_relative=False),
                lambda: abs(self._gyro.get_yaw().value) > 360 * 10,
                self
            ),
            PrintCommand("Yaw Finished"),
            WaitCommand(5),
            
            InstantCommand(self.finalize_calibrate_wheel_circumfrence)
            )

#SysId Callbacks
    
    def logState(self, state):
        if not self.__loggerState:
            self.__loggerState = StringLogEntry(DataLogManager.getLog(),
                                                "sysid-test-state-drive")
        self.__loggerState.append(sysid.SysIdRoutine.stateEnumToString(state))    
                  
    def driveVoltage(self, v):
        self._fl.driveMotor.set_control(VoltageOut(v, enable_foc=True)) 
        self._fr.driveMotor.set_control(VoltageOut(v, enable_foc=True))
        self._bl.driveMotor.set_control(VoltageOut(v, enable_foc=True))
        self._br.driveMotor.set_control(VoltageOut(v, enable_foc=True))
    
    def sysid_quasistatic_cmd(self, direction):
        return self.__sysid.quasistatic(direction)
        
    
    def sysid_dynamic_cmd(self, direction):
        return self.__sysid.dynamic(direction)
     
    def driveLog(self, log: SysIdRoutineLog):
        log.motor("fl") \
            .voltage(self._fl.driveMotor.get_motor_voltage().value) \
            .velocity(self._fl.get_state().speed) \
            .position(self._fl.get_position().distance)
        log.motor("fr") \
            .voltage(self._fr.driveMotor.get_motor_voltage().value) \
            .velocity(self._fr.get_state().speed) \
            .position(self._fr.get_position().distance)
        log.motor("bl") \
            .voltage(self._bl.driveMotor.get_motor_voltage().value) \
            .velocity(self._bl.get_state().speed) \
            .position(self._bl.get_position().distance)
        log.motor("br") \
            .voltage(self._br.driveMotor.get_motor_voltage().value) \
            .velocity(self._br.get_state().speed) \
            .position(self._br.get_position().distance)        