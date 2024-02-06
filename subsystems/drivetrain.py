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

from commands2 import WaitCommand, InstantCommand, FunctionalCommand, PrintCommand
from commands2 import *
from commands2.cmd import *

from commands2.button import CommandXboxController 

from phoenix6.controls import VelocityVoltage, MotionMagicVoltage, VoltageOut

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
from wpilib import DriverStation

from wpimath.filter import SlewRateLimiter

    

class SwerveModule(object):

    def __init__(self, driveCANID, steerCANID, steerCANCoderID, steerInverted):
        self.id = driveCANID

        self.driveMotor = TalonFX(driveCANID, "2481")
        self.steerMotor = TalonFX(steerCANID, "2481")
        self.steerEncoder = CANcoder(steerCANCoderID, "2481")

        self.driveMotorConfig = TalonFXConfiguration()
        self.driveMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.driveMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.driveMotorConfig.slot0.k_p = constants.kdriveP
        self.driveMotorConfig.slot0.k_i = constants.kdriveI 
        self.driveMotorConfig.slot0.k_d = constants.kdriveD
        self.driveMotorConfig.slot0.k_v = constants.kdriveV
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
    
    def set_state(self, state: SwerveModuleState):
        if self.id == 1:
            ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard").putNumber("State_Speed0", state.speed)
            #ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard").putNumber("State_Angle0", state.angle.degrees())
            #ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard").putNumber("Self_Angle1", self.angle().degrees())
        state = SwerveModuleState.optimize(state, self.angle())
        ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard").putNumber("State_Speed1", state.speed)
        self.driveMotor.set_control(VoltageOut(state.speed * 12.0))        
        self.steerMotor.set_control(MotionMagicVoltage(state.angle.degrees() / 360))
    
    def get_position(self):
        return SwerveModulePosition(
            distance=-self.distance(),
            angle=self.angle()
        )
   
    def get_state(self): 
        return SwerveModuleState(
            speed=wpimath.units.inchesToMeters(-self.driveMotor.get_velocity().value * (self.wheel_circumference)),
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
        

        self._gyro = Pigeon2(constants.kPigeonCANID, "2481")

        self.__kinematics = SwerveDrive4Kinematics(
            Translation2d(constants.kWheelBase / 2.0, constants.kWheelTrack / 2.0),
            Translation2d(constants.kWheelBase / 2.0, -constants.kWheelTrack / 2.0),
            Translation2d(-constants.kWheelBase / 2.0, constants.kWheelTrack / 2.0),
            Translation2d(-constants.kWheelBase / 2.0, -constants.kWheelTrack / 2.0))

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

        self.__sd = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")
        
        AutoBuilder.configureHolonomic(
            self.get_pose, # Robot pose supplier
            self.reset_pose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.get_robot_relative_speed, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.drive_robot_relative_speed, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(0.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(0.0, 0.0, 0.0), # Rotation PID constants
                5.5, # Max module speed, in m/s
                0.45, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )
        
        self.leftXRateLimiter = SlewRateLimiter(1)
        self.leftYRateLimiter = SlewRateLimiter(1)
        self.rightXRateLimiter = SlewRateLimiter(1)
        
    
    
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
        
        self.__sd.putNumber("FL_Angle_Actual", self._fl.get_position().angle.degrees())
        self.__sd.putNumber("FL_Distance",self._fl.get_position().distance)
        self.__sd.putNumber("FL_Velocity",self._fl.get_state().speed)
        self.__sd.putNumber("FL_Voltage",self._fl.get_voltage())
        
        self.__sd.putNumber("FR_Angle_Actual", self._fr.get_position().angle.degrees())
        self.__sd.putNumber("FR_Distance",self._fr.get_position().distance)
        self.__sd.putNumber("FR_Velocity",self._fr.get_state().speed)
        self.__sd.putNumber("FR_Voltage",self._fr.get_voltage())
        
        self.__sd.putNumber("BL_Angle_Actual", self._bl.get_position().angle.degrees())
        self.__sd.putNumber("BL_Distance",self._bl.get_position().distance)
        self.__sd.putNumber("BL_Velocity",self._bl.get_state().speed)
        self.__sd.putNumber("BL_Voltage",self._bl.get_voltage())
        
        self.__sd.putNumber("BR_Angle_Actual", self._br.get_position().angle.degrees())      
        self.__sd.putNumber("BR_Distance",self._br.get_position().distance)
        self.__sd.putNumber("BR_Velocity",self._br.get_state().speed)
        self.__sd.putNumber("BR_Voltage",self._br.get_voltage())
        
        self.__sd.putNumber("Yaw", self._gyro.get_yaw().value)
        self.__sd.putNumber("X_POSE", self.get_pose().x)
        self.__sd.putNumber("Y_POSE", self.get_pose().y)  
        

    def get_pose(self):
        return self.__odometry.getPose()
    
    def reset_pose(self, pose=Pose2d()):
            
            self._gyro.set_yaw(0)
            
            self.__odometry.resetPosition(    
                Rotation2d(),
                (
                    self._fl.get_position(),
                    self._fr.get_position(),
                    self._bl.get_position(),
                    self._br.get_position()
                ),
                pose
            )
        #self.__sd.putNumber("Odometry X",self.(pose(X)))
            
    def drive(self, x, y, theta, field_relative, force_angle=False):
        # dead zone for joysticks
        if abs(x) < 0.1:
            x = 0
        if abs(y)< 0.1:
            y = 0 
        if abs(theta) < 0.1 * 0.04:
            theta = 0       
        
        if field_relative:
            chassis_speed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, Rotation2d.fromDegrees(self._gyro.get_yaw().value))
        else:
            chassis_speed = ChassisSpeeds(x, y, theta)

        self.drive_robot_relative_speed(chassis_speed, force_angle)

    def drive_robot_relative_speed(self, chassis_speed: ChassisSpeeds, force_angle=False):
        return 
        #self.__sd.putNumber("Chassis_Speed_Omega0", chassis_speed.omega)

        chassis_speed = ChassisSpeeds.discretize(chassis_speed, constants.kDrivePeriod)
        
        #self.__sd.putNumber("Chassis_Speed_Omega1", chassis_speed.omega)

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
        #self.__sd.putNumber("FL Distance", module_states[0].distance)
        #self.__sd.putNumber("FR Distance", module_states[1].distance)
        #self.__sd.putNumber("BL Distance", module_states[2].distance)
        #self.__sd.putNumber("BR Distance", module_states[3].distance)
        

        

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
            lambda: self.drive(self.leftYRateLimiter.calculate(joystick.getLeftY()),
                            self.leftXRateLimiter.calculate(joystick.getLeftX()),
                            self.rightXRateLimiter.calculate(joystick.getRightX() * .04),
                               True
                                ),
            lambda: self.drive(0, 0, 0, True), 
            self
        )
    
    def drive_with_joystick_limelight_align_cmd(self, joystick: CommandXboxController):
        return runEnd(
             
            lambda: self.drive(self.leftYRateLimiter.calculate(joystick.getLeftY()),
                               self.leftXRateLimiter.calculate(joystick.getLeftX()),
                               ntcore.NetworkTableInstance.getTable("limelight").getNumber('tx'),
                               False
                                ),
            lambda: self.drive(0, 0, 0, False), 
            self
        )
    
    def line_up_with_joystick_limelight_align_cmd(self, joystick: CommandXboxController):
        return runEnd(             
            lambda: self.drive(self.leftYRateLimiter.calculate(joystick.getLeftY()),
                               ntcore.NetworkTableInstance.getTable("limelight").getNumber('ty'),
                               self.rightXRateLimiter.calculate(joystick.getRightX() * .04),
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
                
    def zero_drive_encoder_cmd(self):
        return runOnce(self.zero_drive_encoder).ignoringDisable(True)

    def zero_steer_encoder(self):
        wpilib.Preferences.setDouble("FL_STEER_OFFSET", self._fl.zero_steer_encoder())
        wpilib.Preferences.setDouble("FR_STEER_OFFSET", self._fr.zero_steer_encoder())
        wpilib.Preferences.setDouble("BL_STEER_OFFSET", self._bl.zero_steer_encoder())
        wpilib.Preferences.setDouble("BR_STEER_OFFSET", self._br.zero_steer_encoder())
        
    def zero_steer_encoder_cmd(self):
        return runOnce(self.zero_steer_encoder).ignoringDisable(True)
   
    
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
            
            # Point all wheels at a ~45 deg.
            InstantCommand(
                lambda: self.drive(0.0, 0.0, theta=1.0, field_relative=False,force_angle=True)), 
            WaitCommand(1),

            # Keep track of the wheel distance at the beginning of the cal process.
            InstantCommand(self.zero_drive_encoder),
            PrintCommand ("Zero Drive Encoder"),

            FunctionalCommand(
                lambda: None,
                lambda: self.drive(0.0, 0.0, theta=0.01, field_relative=False),
                lambda interupted: self.drive(0.0, 0.0, 0.0, field_relative=False),
                lambda: abs(self._gyro.get_yaw().value) > 360 * 5,
                self
            ),
            PrintCommand("Yaw Finished"),
            WaitCommand(5),
            
            InstantCommand(self.finalize_calibrate_wheel_circumfrence)
            
        
        )
        

        