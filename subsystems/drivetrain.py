import math
import json
import wpilib
import ntcore
import wpimath.units
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry, \
ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.estimator import SwerveDrive4PoseEstimator
#Pathplanner stuff
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpilib import DriverStation
from pathplannerlib.auto import PathPlannerAuto
from wpilib import DriverStation

from commands2 import WaitCommand, InstantCommand, FunctionalCommand, PrintCommand
from commands2 import *
from commands2 import sysid
from commands2.cmd import *

from commands2.button import CommandXboxController 

from ntcore import NetworkTableInstance

from phoenix6.controls import VelocityVoltage, MotionMagicVoltage, VoltageOut, DutyCycleOut

from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.cancoder_configs import CANcoderConfiguration 

from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware import TalonFX
from phoenix6.hardware import Pigeon2

import constants

import phoenix6
from phoenix6.signals.spn_enums import *


from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants

from wpilib.sysid import SysIdRoutineLog
from wpiutil.log import StringLogEntry
from wpilib import DataLogManager
from wpilib import SmartDashboard, Field2d

from utils import *

   # hi from 2024 

class SwerveModule(object):

    def __init__(self, driveCANID, steerCANID, steerCANCoderID, steerInverted):
        self.id = driveCANID

        self.driveMotor = TalonFX(driveCANID, "2481")
        self.steerMotor = TalonFX(steerCANID, "2481")
        self.steerEncoder = CANcoder(steerCANCoderID, "2481")

        self.driveMotorConfig = TalonFXConfiguration()
        self.driveMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.driveMotorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.driveMotorConfig.slot0.k_p = constants.kdriveP
        self.driveMotorConfig.slot0.k_i = constants.kdriveI 
        self.driveMotorConfig.slot0.k_d = constants.kdriveD
        self.driveMotorConfig.slot0.k_v = constants.kdriveV
        self.driveMotorConfig.slot0.k_a = constants.kdriveA
        self.driveMotorConfig.slot0.k_s = constants.kdriveS
        self.driveMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kSwerveSteerCruiseVelocity
        self.driveMotorConfig.motion_magic.motion_magic_acceleration = constants.kSwerveSteerAcceleration 
        self.driveMotorConfig.feedback.sensor_to_mechanism_ratio = constants.kSwerveReductionDrive
        self.driveMotor.configurator.apply(self.driveMotorConfig)
        
        self.steerMotorConfig = TalonFXConfiguration()
        self.steerMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.steerMotorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if steerInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.steerMotorConfig.motor_output.peak_forward_duty_cycle = 0.2
        self.steerMotorConfig.motor_output.peak_reverse_duty_cycle = -0.2
        self.steerMotorConfig.slot0.k_p = constants.ksteerP
        self.steerMotorConfig.slot0.k_i = constants.ksteerI
        self.steerMotorConfig.slot0.k_d = constants.ksteerD
        self.steerMotorConfig.slot0.k_v = constants.ksteerV
        self.steerMotorConfig.slot0.k_a = constants.ksteerA
        self.steerMotorConfig.slot0.k_s = constants.ksteerS
        self.steerMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kSwerveSteerCruiseVelocity
        self.steerMotorConfig.motion_magic.motion_magic_acceleration = constants.kSwerveSteerAcceleration
        self.steerMotorConfig.motion_magic.motion_magic_jerk = constants.kSwerveSteerJerk
        self.steerMotorConfig.closed_loop_general.continuous_wrap = True

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
        print("Zero Encoder Start")
        #zeros the offset so we dont have to deal with the previous one
        self.canCoderConfig.magnet_sensor.magnet_offset = 0
        self.steerEncoder.configurator.apply(self.canCoderConfig)
        
        #Wait for fresh data after offset
        steer_offset = self.steerEncoder.get_absolute_position()
        steer_offset.wait_for_update(1)
        self.canCoderConfig.magnet_sensor.magnet_offset = -steer_offset.value
        self.steerEncoder.configurator.apply(self.canCoderConfig)
        print("Zero Encoder Finish")
        return -steer_offset.value
        
    def set_steer_offset(self, steer_offset:float):
        self.canCoderConfig.magnet_sensor.magnet_offset = steer_offset
        self.steerEncoder.configurator.apply(self.canCoderConfig)
       
    def distance(self):
        return wpimath.units.inchesToMeters(self.driveMotor.get_position().value * self.wheel_circumference)
    
    
    def angle(self):
        return Rotation2d(self.steerEncoder.get_absolute_position().value * 2 * math.pi)
    
    def set_state(self, state: SwerveModuleState, voltage_only):
        state = SwerveModuleState.optimize(state, self.angle())
        if voltage_only:
            self.driveMotor.set_control(DutyCycleOut(state.speed / constants.kDriveMaxSpeed, True))
        else:
            self.driveMotor.set_control(VelocityVoltage(wpimath.units.metersToInches(state.speed) / self.wheel_circumference))        
        
        if abs(state.speed) < 0.05:
            self.steerMotor.set_control(VoltageOut(0.0)) 
        else:   
            self.steerMotor.set_control(MotionMagicVoltage(state.angle.degrees() / 360))
    
    def get_position(self):
        return SwerveModulePosition(
            #negative sign is to fix inverted Odometry
            distance=self.distance(),
            angle=self.angle()
        )
   
    def get_state(self): 
        return SwerveModuleState(
            speed=wpimath.units.inchesToMeters(self.driveMotor.get_velocity().value * (self.wheel_circumference)),
            angle=self.angle()
        )
    
    def zero_drive_encoder(self):
       self.driveMotor.set_position(0)
    
    def get_voltage(self):
        return self.driveMotor.get_motor_voltage().value
       

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

        SmartDashboard.putNumber("limelight gain", 0.2)
        
        AutoBuilder.configureHolonomic(
            self.get_pose, # Robot pose supplier
            self.reset_pose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.get_robot_relative_speed, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.drive_robot_relative_speed, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(2.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(2.0, 0.0, 0.0), # Rotation PID constants
                6.01, # Max module speed, in m/s
                0.45, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )
          
        self.__loggerState = None
        
        self.__sysid_config = sysid.SysIdRoutine.Config(recordState=self.logState)
        self.__sysid_mechanism = sysid.SysIdRoutine.Mechanism(self.driveVoltage, self.driveLog, self, "drive")
        
        self.__sysid = sysid.SysIdRoutine(self.__sysid_config, self.__sysid_mechanism)
        
        self.ll_rear_table = NetworkTableInstance.getDefault().getTable("limelight-rear")
    
        self.drive_state = True
        
    def shouldFlipPath(self):
            # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kBlue
    
    def periodic(self):
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
        
    # Lime Light Update 
    def limelight_periodic(self):
        ll_json = json.loads(self.ll_rear_table.getString("json", "{}"))

        num_targets = 0 
        if "Results" in ll_json and "Fiducial" in ll_json["Results"]:
            num_targets = len(ll_json["Results"]["Fiducial"])
         
        #checks if april tag is visible
        if self.ll_rear_table.getNumber("tv",0) > 0:
            bot_pose = self.ll_rear_table.getEntry("botpose_wpiblue").getDoubleArray([0,0,0,0,0,0])
            total_latency_ms = self.ll_rear_table.getNumber("cl",0) + \
                               self.ll_rear_table.getNumber("tl",0) 
            capture_timestamp_sec = wpilib.Timer.getFPGATimestamp() - total_latency_ms / 1000.0
            vision_pose = Pose2d(x=bot_pose[0],
                                 y=bot_pose[1],
                                 rotation=Rotation2d.fromDegrees(bot_pose[5]))

            if (vision_pose.X() == 0.0): 
                return

            distance_to_pose = self.get_pose().relativeTo(vision_pose).translation().norm()
            if num_targets:
                xyStds = 0
                degStds = 0
                
                if (num_targets >= 2): 
                    xyStds = 0.5
                    degStds = 6
      
                elif self.ll_rear_table.getNumber("ta",0) > 0.8 and distance_to_pose < 0.5:
                    xyStds = 1.0;
                    degStds = 12;
                    
                elif self.ll_rear_table.getNumber("ta",0) > 0.1 and distance_to_pose < 0.3:
                    xyStds = 2.0;
                    degStds = 30;           
                
                if xyStds > 0:
                    self.__odometry.setVisionMeasurementStdDevs((xyStds, xyStds, math.radians(degStds)))
                    self.__odometry.addVisionMeasurement(vision_pose, capture_timestamp_sec)
      
     
    def dashboard_periodic(self):                                                       
        SmartDashboard.putNumber("FL_Angle_Actual", self._fl.get_position().angle.degrees())
        SmartDashboard.putNumber("FL_Distance",self._fl.get_position().distance)
        SmartDashboard.putNumber("FL_Velocity",self._fl.driveMotor.get_rotor_velocity().value)
        SmartDashboard.putNumber("FL_Voltage",self._fl.get_voltage())
        SmartDashboard.putNumber("FL Duty Cycle", self._fl.driveMotor.get_duty_cycle().value)
        SmartDashboard.putNumber("FL Current", self._fl.driveMotor.get_supply_current().value)        
        
        SmartDashboard.putNumber("FR_Angle_Actual", self._fr.get_position().angle.degrees())
        SmartDashboard.putNumber("FR_Distance",self._fr.get_position().distance)
        SmartDashboard.putNumber("FR_Velocity",self._fr.driveMotor.get_rotor_velocity().value)
        SmartDashboard.putNumber("FR_Voltage",self._fr.get_voltage())
        SmartDashboard.putNumber("FR Duty Cycle", self._fr.driveMotor.get_duty_cycle().value)
        SmartDashboard.putNumber("FR Current", self._fr.driveMotor.get_supply_current().value)
        
        SmartDashboard.putNumber("BL_Angle_Actual", self._bl.get_position().angle.degrees())
        SmartDashboard.putNumber("BL_Distance",self._bl.get_position().distance)
        SmartDashboard.putNumber("BL_Velocity",self._bl.driveMotor.get_rotor_velocity().value)
        SmartDashboard.putNumber("BL_Voltage",self._bl.get_voltage())
        SmartDashboard.putNumber("BL Duty Cycle", self._bl.driveMotor.get_duty_cycle().value)
        SmartDashboard.putNumber("BL Current", self._bl.driveMotor.get_supply_current().value)
        
        SmartDashboard.putNumber("BR_Angle_Actual", self._br.get_position().angle.degrees())      
        SmartDashboard.putNumber("BR_Distance",self._br.get_position().distance)
        SmartDashboard.putNumber("BR_Velocity",self._br.driveMotor.get_rotor_velocity().value)
        SmartDashboard.putNumber("BR_Voltage",self._br.get_voltage())
        SmartDashboard.putNumber("BR Duty Cycle", self._br.driveMotor.get_duty_cycle().value)
        SmartDashboard.putNumber("BR Current", self._fr.driveMotor.get_supply_current().value)
        
        SmartDashboard.putNumber("BR Supply Voltage", self._br.driveMotor.get_supply_voltage().value)
        SmartDashboard.putNumber("FR Supply Voltage", self._fr.driveMotor.get_supply_voltage().value)
        SmartDashboard.putNumber("FL Supply Voltage", self._fl.driveMotor.get_supply_voltage().value)
        SmartDashboard.putNumber("BL Supply Voltage", self._bl.driveMotor.get_supply_voltage().value)
        
        SmartDashboard.putNumber("Yaw", self._gyro.get_yaw().value)
        SmartDashboard.putNumber("X_POSE", self.get_pose().x)
        SmartDashboard.putNumber("Y_POSE", self.get_pose().y)
        
               
        self.field.setRobotPose(self.__odometry.getEstimatedPosition())      
        
    # Odometry

    def get_pose(self) -> Pose2d:
        return self.__odometry.getEstimatedPosition()
           
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
            
    def reset_odom_to_vision(self):
        bot_pose = self.ll_rear_table.getEntry("botpose.wpiblue").getDoubleArray([0,0,0,0,0,0])

        vision_pose = Pose2d(x=bot_pose[0],
                             y=bot_pose[1],
                             rotation=Rotation2d.fromDegrees(bot_pose[3]))
        self.reset_pose(vision_pose)
    
    def reset_odom_to_vision_cmd(self):
        return runOnce (self.reset_odom_to_vision)
    
    def get_range_to_speaker(self):
        return  5 #self.get_pose().relativeTo(Translation2d(x=-0.0381, y=5.547)).translation().norm()
            
    
    # Drive Controls
                
    def drive(self, x, y, theta, field_relative, force_angle=False):                
        if field_relative:
            chassis_speed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, Rotation2d.fromDegrees(self._gyro.get_yaw().value))
        else:
            chassis_speed = ChassisSpeeds(x, y, theta)

        self.drive_robot_relative_speed(chassis_speed, force_angle, True)

    def toggle_robot_relative_driving(self):
        self.robot_relative_driving = not self.robot_relative_driving
        
    def toggle_robot_relative_driving_cmd(self):
        return runOnce(self.toggle_robot_relative_driving)
    
    def field_centric_cmd(self):
        self.drive_state = True
        
    def robot_centric_cmd(self):
        self.drive_state = False
            
    def drive_robot_relative_speed(self, chassis_speed: ChassisSpeeds, force_angle=False, voltage_only=False):
        SmartDashboard.putNumber("Target Omega", chassis_speed.omega)
        SmartDashboard.putNumber("Chassis Speed X", chassis_speed.vx)
        SmartDashboard.putNumber("Chassis Speed Y", chassis_speed.vy)

        #chassis_speed = ChassisSpeeds.discretize(chassis_speed, constants.kDrivePeriod) -Should be using

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
        
        SmartDashboard.putNumber("Actual Omega", cs.omega)
        SmartDashboard.putNumber("Actual Vx", cs.vx)
        SmartDashboard.putNumber("Actual Vy", cs.vy)
        
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
    
    def limelight_angulor_alignment_cmd(self, joystick: CommandXboxController):
        return RepeatCommand(
                SequentialCommandGroup(
                    self.drive_with_joystick_cmd(joystick).raceWith(self.wait_for_note_visible()),
                    self.drive_towards_note_command(joystick).raceWith(self.wait_for_no_note_visible())          
            )   
        )
        
    def drive_with_joystick_limelight_target_align_cmd(self, joystick: CommandXboxController):
        return RepeatCommand(
                SequentialCommandGroup(
                    self.drive_with_joystick_cmd(joystick).raceWith(self.wait_for_target_visible()),
                    self.drive_towards_target_command(joystick).raceWith(self.wait_for_no_target_visible())          
            )   
        )
    
    def line_up_with_april_tag_cmd(self, joystick: CommandXboxController):
        return RepeatCommand(             
                SequentialCommandGroup(
                    self.drive_with_joystick_cmd(joystick).raceWith(self.wait_for_note_visible()),
                    self.amp_lineup_cmd(joystick).raceWith(self.wait_for_no_note_visible())                               
            )              
        )
        
    def wait_for_note_visible(self):
        return WaitUntilCommand(lambda: NetworkTableInstance.getDefault().getTable("limelight-front").getNumber('tv', 0) == 1)
    
    def wait_for_no_note_visible(self):
        return WaitUntilCommand(lambda: NetworkTableInstance.getDefault().getTable("limelight-front").getNumber('tv', 0) == 0)
        
    def drive_towards_note_command(self, joystick: CommandXboxController):
        return runEnd( 
            lambda: self.drive(scale_axis(math.hypot(joystick.getLeftY(), joystick.getLeftX())) * constants.kDriveMaxSpeed,
                                0,
                                scale_axis(-NetworkTableInstance.getDefault().getTable("limelight-front").getNumber('tx', 0) *
                                SmartDashboard.getNumber("limelight gain", 0)),
                                False
                    ), 
            lambda: None,
            self
        )
    
    def drive_towards_target_command(self, joystick: CommandXboxController):
        return runEnd( 
            lambda: self.drive(scale_axis(-joystick.getLeftY()) * constants.kDriveMaxSpeed,
                            scale_axis(-joystick.getLeftX()) * constants.kDriveMaxSpeed,
                                -NetworkTableInstance.getDefault().getTable("limelight-rear").getNumber('tx', 0) *
                                SmartDashboard.getNumber("limelight gain", 0),
                                True
                    ), 
            lambda: None,
            self
        )
    
    def amp_lineup_cmd(self, joystick: CommandXboxController):
        return runEnd(
            lambda: self.drive(scale_axis(-joystick.getLeftY() * constants.kDriveMaxSpeed),
                               -NetworkTableInstance.getDefault().getTable("limelight-front").getNumber('tx', 0) * 
                                   SmartDashboard.getNumber("limelight gain", 0),
                               scale_axis(-joystick.getRightX() * 6),
                               True
                               ),
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
        self._fl.driveMotor.set_control(VoltageOut(v, enable_foc=False)) 
        self._fr.driveMotor.set_control(VoltageOut(v, enable_foc=False))
        self._bl.driveMotor.set_control(VoltageOut(v, enable_foc=False))
        self._br.driveMotor.set_control(VoltageOut(v, enable_foc=False))
    
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