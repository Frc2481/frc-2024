import math
import wpilib
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry, \
ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d, Pose2d

import commands2
import commands2.cmd

import constants

import phoenix6
from phoenix6.signals.spn_enums import *


class SwerveModule(object):

    def __init__(self, driveCANID, steerCANID, steerCANCoderID):

        self.driveMotor = phoenix6.TalonFX(driveCANID)
        self.steerMotor = phoenix6.TalonFX(steerCANID)
        self.steerEncoder = phoenix6.Cancoder(steerCANCoderID)

        self.driveMotorConfig = phoenix6.TalonFXConfiguration()
        self.driveMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.driveMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.driveMotorConfig.slot0.k_p = constants.kdriveP
        self.driveMotorConfig.slot0.k_i = constants.kdriveI 
        self.driveMotorConfig.slot0.k_d = constants.kdriveD
        self.driveMotorConfig.slot0.k_f = constants.kDriveMotorF 
        self.driveMotorConfig.feedback.sensor_to_mechanism_ratio = constants.kSwerveReductionDrive
        self.driveMotor.configuration.apply(self.driveMotorConfig)
        
        self.steerMotorConfig = phoenix6.TalonFXConfiguration()
        self.steerMotorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.steerMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.steerMotorConfig.slot0.k_p = constants.ksteerP
        self.steerMotorConfig.slot0.k_i = constants.ksteerI
        self.steerMotorConfig.slot0.k_d = constants.ksteerD
        self.steerMotorConfig.slot0.k_f = constants.ksteerF

        self.steerMotorConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.Fused_CANCODER
        self.steerMotorConfig.feedback.feedback_remote_sensor_id = steerCANCoderID 
        self.steerMotorConfig.feedback.sensor_to_mechanism_ration = 1.0
        self.steerMotorConfig.feedback.rotor_to_sensor_ratio = constants.kSwerveReductionSteer
        self.steerMotor.configurator.apply(self.steerMotorConfig)

        self.canCoderConfig = phoenix6.CANcoderConfiguration()
        self.canCoderConfig.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        self.canCoderConfig.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        self.canCoderConfig.magnet_sensor.magnet_offset = 0.4
        self.steerEncoder.configuration.apply(self.canCoderConfig)

    def distance(self):
        return self.driveMotor.getposition().value
    
    def angle(self):
        return Rotation2d(self.steerEncoder.get_position().value * 2 * math.pi)
    
    def set_state(self, state):
        state = SwerveModuleState.optimize(state, self.angle())
        self.driveMotor.set_control(phoenix6.VelocityVoltage(state.speed))
        self.steerMotor.set_control(phoenix6.VelocityVoltage(state.angle))
    
    def get_state(self):
        return SwerveModulePosition(
            distance=self.distance(),
            angle=self.angle()
        )


class DriveSubsystem(commands2.SubsystemBase):

    def __init__(self):
        super().__init__()

        self.__fl = SwerveModule(constants.kSwerveFrontLeftDriveMotorCANID, 
                                 constants.kSwerveFrontLeftSteerMotorCANID, 
                                 constants.kSwerveFrontLeftSteerEncoderCANID)  
        self.__fr = SwerveModule(constants.kSwerveFrontRightDriveMotorCANID, 
                                 constants.kSwerveFrontRightSteerMotorCANID,
                                 constants.kSwerveFrontRightSteerEncoderCANID)
        self.__bl = SwerveModule(constants.kSwerveBackLeftDriveMotorCANID,
                                 constants.kSwerveBackLeftSteerMotorCANID, 
                                 constants.kSwerveBackLeftSteerEncoderCANID)
        self.__br = SwerveModule(constants.kSwerveBackRightDriveMotorCANID,
                                 constants.kSwerveBackRightSteerMotorCANID, 
                                 constants.kSwerveBackrightSteerEncoderCANID) 

        self.__gyro = phoenix6.Pigeon2(constants.kPigeonCANID)

        self.__kinematics = SwerveDrive4Kinematics(
            Translation2d(-constants.kWheelTrack / 2.0, constants.kWheelBase / 2.0),
            Translation2d(constants.kWheelTrack / 2.0, constants.kWheelBase / 2.0),
            Translation2d(-constants.kWheelTrack / 2.0, -constants.kWheelBase / 2.0),
             Translation2d(constants.kWheelTrack / 2.0, -constants.kWheelBase / 2.0),
                                                                                    
        )      
        self.odometry = SwerveDrive4Odometry(
            self.__kinematics,
            self.__gyro.get_yaw().value,
            (
                self.__fl.get_state(),
                self.__fr.get_state(),
                self.__bl.get_state(),
                self.__br.get_state()
            ),
            Pose2d()
        )
    def periodic(self):

        self.__odometry.update(
            self.__gyro.get_yaw().value,
            self.__fl.get_state(),
            self.__fr.get_state(),
            self.__bl.get_state(),
            self.__br.get_state(),
        )

    def get_pose(self):
        return self.__odometry.getPose()
    
    def reset_pose(self, pose):
            self.__odometry.resetPosition(
                self.__gyro.get_yaw().value,
                self.__fl.get_state(),
                self.__fr.get_state(),
                self.__bl.get_state(),
                self.__br.get_state(),
                pose
            )
    def drive(self, x, y, theta, field_relative):
        if field_relative:
            chassis_speed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, self.__gyro.get_yaw().value)
        else:
            chassis_speed = ChassisSpeeds(x, y, theta)

        chassis_speed = ChassisSpeeds.discretize(chassis_speed, constants.kDrivePeriod)

        module_states = self.__kinematics.toSwerveModuleStates(chassis_speed)

        module_states = self.__kinematics.desaturateWheelSpeeds(module_states, constants.kDriveMaxSpeed)

        self.__fl.set_state(module_states[0])
        self.__fr.set_state(module_states[1])
        self.__bl.set_state(module_states[2])
        self.__br.set_state(module_states[3])


        
        

                   
                   



                                                            


    
                          
    










