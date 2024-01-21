"""Module defines SwerveModules and Drivetrain for 2024 FRC2481 Robot"""
import math
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d, Pose2d

import commands2
import commands2.cmd

from phoenix6.controls import VelocityVoltage
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware import TalonFX
from phoenix6.hardware import Pigeon2
from phoenix6.signals import spn_enums

import constants


class SwerveModule:
    """Module for an individual swerve module (one corner of the bot)"""

    def __init__(self, drive_can_id, steer_can_id, steer_can_coder_id):

        self.drive_motor = TalonFX(drive_can_id)
        self.steer_motor = TalonFX(steer_can_id)
        self.steer_encoder = CANcoder(steer_can_coder_id)

        self.drive_motor_config = TalonFXConfiguration()
        self.drive_motor_config.motor_output.neutral_mode = spn_enums.NeutralModeValue.BRAKE
        self.drive_motor_config.motor_output.inverted = \
            spn_enums.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.drive_motor_config.slot0.k_p = constants.DRIVE_P
        self.drive_motor_config.slot0.k_i = constants.DRIVE_I
        self.drive_motor_config.slot0.k_d = constants.DRIVE_D
        self.drive_motor_config.slot0.k_f = constants.DRIVE_F
        self.drive_motor_config.feedback.sensor_to_mechanism_ratio = \
            constants.SWERVE_REDUCTION_DRIVE
        self.drive_motor.configurator.apply(self.drive_motor_config)

        self.steer_motor_config = TalonFXConfiguration()
        self.steer_motor_config.motor_output.neutral_mode = spn_enums.NeutralModeValue.BRAKE
        self.steer_motor_config.motor_output.inverted = \
            spn_enums.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.steer_motor_config.slot0.k_p = constants.STEER_P
        self.steer_motor_config.slot0.k_i = constants.STEER_I
        self.steer_motor_config.slot0.k_d = constants.STEER_D
        self.steer_motor_config.slot0.k_f = constants.STEER_F

        self.steer_motor_config.feedback.feedback_sensor_source = \
            spn_enums.FeedbackSensorSourceValue.FUSED_CANCODER
        self.steer_motor_config.feedback.feedback_remote_sensor_id = steer_can_coder_id
        self.steer_motor_config.feedback.sensor_to_mechanism_ratio = 1.0
        self.steer_motor_config.feedback.rotor_to_sensor_ratio = constants.SWERVE_REDUCTION_STEER
        self.steer_motor.configurator.apply(self.steer_motor_config)

        self.can_coder_config = CANcoderConfiguration()
        self.can_coder_config.magnet_sensor.absolute_sensor_range = \
            spn_enums.AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        self.can_coder_config.magnet_sensor.sensor_direction = \
            spn_enums.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        self.can_coder_config.magnet_sensor.magnet_offset = 0.4
        self.steer_encoder.configurator.apply(self.can_coder_config)

    def distance(self):
        """Return motor position"""
        return self.drive_motor.get_position().value

    def angle(self):
        """Return motor angle"""
        return Rotation2d(self.steer_encoder.get_position().value * 2 * math.pi)

    def set_state(self, state: SwerveModuleState):
        """"Set drive and steer motor state."""
        state = SwerveModuleState.optimize(state, self.angle())
        self.drive_motor.set_control(VelocityVoltage(state.speed))
        self.steer_motor.set_control(VelocityVoltage(state.angle))

    def get_position(self) -> SwerveModulePosition:
        """Get SwerveModulePosition"""
        return SwerveModulePosition(
            distance=self.distance(),
            angle=self.angle()
        )


class DriveSubsystem(commands2.SubsystemBase):
    """Drive Subsystem combining 4 swervemodules to make a drivetrain"""
    def __init__(self):
        super().__init__()

        self.__fl = SwerveModule(constants.SWERVE_FRONT_LEFT_DRIVE_MOTOR_CAN_ID,
                                 constants.SWERVE_FRONT_LEFT_STEER_MOTOR_CAN_ID,
                                 constants.SWERVE_FRONT_LEFT_STEER_ENCODER_CAN_ID)
        self.__fr = SwerveModule(constants.SWERVE_FRONT_RIGHT_DRIVE_MOTOR_CAN_ID,
                                 constants.SWERVE_FRONT_RIGHT_STEER_MOTOR_CAN_ID,
                                 constants.SWERVE_FRONT_RIGHT_STEER_ENCODER_CAN_ID)
        self.__bl = SwerveModule(constants.SWERVE_BACK_LEFT_DRIVE_MOTOR_CAN_ID,
                                 constants.SWERVE_BACK_LEFT_STEER_MOTOR_CAN_ID,
                                 constants.SWERVE_BACK_LEFT_STEER_ENCODER_CAN_ID)
        self.__br = SwerveModule(constants.SWERVE_BACK_RIGHT_DRIVE_MOTOR_CAN_ID,
                                 constants.SWERVE_BACK_RIGHT_STEER_MOTOR_CAN_ID,
                                 constants.SWERVE_BACK_RIGHT_STEER_ENCODER_CAN_ID)

        self.__gyro = Pigeon2(constants.PIGEON_GYRO_CAN_ID)

        self.__kinematics = SwerveDrive4Kinematics(
            Translation2d(-constants.DRIVE_WHEEL_TRACK / 2.0, constants.DRIVE_WHEEL_BASE / 2.0),
            Translation2d(constants.DRIVE_WHEEL_TRACK / 2.0, constants.DRIVE_WHEEL_BASE / 2.0),
            Translation2d(-constants.DRIVE_WHEEL_TRACK / 2.0, -constants.DRIVE_WHEEL_BASE / 2.0),
             Translation2d(constants.DRIVE_WHEEL_TRACK / 2.0, -constants.DRIVE_WHEEL_BASE / 2.0),
        )

        self.__odometry = SwerveDrive4Odometry(
            kinematics = self.__kinematics,
            gyroAngle = Rotation2d(),
            modulePositions=(
                self.__fl.get_position(),
                self.__fr.get_position(),
                self.__bl.get_position(),
                self.__br.get_position()
            ),
            initialPose=Pose2d()
        )

    def periodic(self):
        """Period Update"""
        self.__odometry.update(
            self.__gyro.get_yaw().value,
            self.__fl.get_position(),
            self.__fr.get_position(),
            self.__bl.get_position(),
            self.__br.get_position(),
        )

    def get_pose(self):
        """Get Pose"""
        return self.__odometry.getPose()

    def reset_pose(self, pose):
        """Reset Pose"""
        self.__odometry.resetPosition(
            self.__gyro.get_yaw().value,
            self.__fl.get_position(),
            self.__fr.get_position(),
            self.__bl.get_position(),
            self.__br.get_position(),
            pose
        )

    def drive(self, x, y, theta, field_relative):
        """Drive method"""
        if field_relative:
            chassis_speed = \
                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, self.__gyro.get_yaw().value)
        else:
            chassis_speed = ChassisSpeeds(x, y, theta)

        chassis_speed = ChassisSpeeds.discretize(chassis_speed, constants.DRIVE_PERIOD)

        module_states = self.__kinematics.toSwerveModuleStates(chassis_speed)

        module_states = \
            self.__kinematics.desaturateWheelSpeeds(module_states, constants.DRIVE_MAX_SPEED)

        self.__fl.set_state(module_states[0])
        self.__fr.set_state(module_states[1])
        self.__bl.set_state(module_states[2])
        self.__br.set_state(module_states[3])
