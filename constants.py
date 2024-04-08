import math
import wpimath.units
from wpimath.geometry import Pose2d, Rotation2d

kSwerveFrontLeftDriveMotorCANID = 1
kSwerveFrontLeftSteerMotorCANID = 2
kSwerveFrontRightDriveMotorCANID = 3
kSwerveFrontRightSteerMotorCANID = 4
kSwerveBackLeftDriveMotorCANID = 5
kSwerveBackLeftSteerMotorCANID = 6
kSwerveBackRightDriveMotorCANID = 7
kSwerveBackRightSteerMotorCANID = 8
kShooterMotorCANID = 9
kIntakeVerticalMotorCANID = 10
kIntakeHorizontalMotorCANID = 11
kFeederMotorCANID = 12
kAngulatorMotorCANID = 30
kArmMotorCANID = 14
kSwerveFrontLeftSteerEncoderCANID = 2
kSwerveFrontRightSteerEncoderCANID = 4
kSwerveBackLeftSteerEncoderCANID = 6
kSwerveBackrightSteerEncoderCANID = 8
kAngulatorEncoderCANID = 31
kPigeonCANID = 1

kDriveMaxSpeed = 5.7 #6.4 
kDrivePeriod = 0.2

kWheelTrack = wpimath.units.inchesToMeters(18.5)
kWheelBase = wpimath.units.inchesToMeters(16.5)

#wheeelly wheel
kDriveWheelRadiusIn = 1.5
kDistanceTraveledOneRotation = (kDriveWheelRadiusIn * 2 * math.pi) 
kDriveBaseRadiusIn = wpimath.units.metersToInches(((kWheelTrack**2 + kWheelBase**2)**0.5)/2)

kMaxModuleSpeedFt = 19

kDrivePathTranslationKP = 5.0
kDrivePathTranslationKI = 0.0
kDrivePathTranslationKD = 0.0

kDrivePathAngularKP = 5.0
kDrivePathAngularKI = 0.0
kDrivePathAngularKD = 0.0

#drivetrain
#kdriveP = 1.5
kdriveP = 1.5
kdriveI = 0
kdriveD = 0.0
#kdriveV = 2.0825
kdriveV = 1.8647 * wpimath.units.inchesToMeters(kDistanceTraveledOneRotation)
#kdriveA = 0.24925
kdriveA = 0.312 * wpimath.units.inchesToMeters(kDistanceTraveledOneRotation)
kdriveS = 0.07
kDriveGearReduction = 125

kAlignStateNone = 0
kAlignStateSpeaker = 1
kAlignStateAmp = 2
kAlignStateNote = 3

#steer
ksteerP = 100
ksteerI = 0.1
ksteerD = 0.0
ksteerV = 3.42
ksteerA = 0
ksteerS = 0
kSwerveSteerCruiseVelocity = 3.5
kSwerveSteerAcceleration = 24
kSwerveSteerJerk = 0

kSwerveReductionSteer = 30
kSwerveReductionDrive = 3.33 #4.31

kDriverControllerPort = 0
kOperatorControllerPort = 1
kDiagControllerPort = 2

#Feeder
kFeederSpeedRPS = 10
kFeederP = 5
kFeederI = 1
kFeederD = 0
kFeederV = 1.0
kTeleopFeederSpeed = 0.2 #0.2
kTeleopFeederSpeedRaisedAuto = 0.5

#BeamBrake
kFeederBeambreakStageTwoPort = 9
kFeederBeambreakStageOnePort = 7
kArmZero = 8

#ShooterFocCurrentGains
kShooterSpeedFeedRPS = 50
kShooterSpeedSubwooferRPS = 65
kShooterSpeedHappyDonutRPS = 82
kShooterSpeedMaxRPS = 82
kShooterReverseSpeed = -10
kShooterToArmSpeedRPS = 10
kShooterP = 0.1
kShooterI = 0
kShooterD = 0
kShooterV = 0.0125
kShooterA = 0
kShooterS = 0.02
kShooterPeakCurrent = 40
kShooterCruiseVelocity = 0
kShooterAcceleration = 0
kShooterJerk = 0
kShooterOnTarget = 5

#Gripper
kGripperSolenoidModule = 0
kGripperDoubleSolenoidForwardPort = 6
kGripperDoubleSolenoidReversePort = 7

#Climber
kClimberSolenoidModule = 0
kClimberDoubleSolenoidForwardPort = 4
kClimberDoubleSolenoidReversePort = 5


#Arm Motor
kArmP = 1
kArmI = 0.0
kArmD = 0
kArmV = 0.133
kArmA = 0.0
kArmS = 0.0
kArmAcceleration =800
kArmCruiseVelocity = 90
kArmScorePosition = 32.5
kArmClimbPosition = 34
kArmPickupPosition = 11.7
kArmDownPosition = 1

#angulator
kAngulatorHappyDonutAngleDeg = 0.002
kAngulatorSubwooferAngleDeg = 31.64 / 360
kAngulatorFeedAngleDeg = 0.05
kAngulatorGearReduction = 259.9
kAngulatorDownPosition = 0 # 20.5/360   
kAngulatorUpPosition =  0.112 #/360
kAngulatorP = 150
kAngulatorI = 20# .01
kAngulatorD = 0
kAngulatorV = 30.7 # 0.041
kAngulatorA = 0.00 # 0.0025
kAngulatorS = 0.3
kAngulatorJerk = 0
kAngulatorAcceleration = 2.0
kAngulatorCruiseVelocity = 0.31
kMotorResistance = 6
kAngulatorForwardSoftLimitRot = 0.133 # 70 / 360
kAngulatorReverseSoftLimitRot = 0.00488 # 22.5 / 360
kAngulatorOnTarget = 0.001

#Metric system units
kArmMass = 10 / 2.205
kArmLength = 16 / 39.37 

#steer
kSteerGearReduction = 4

#intake
kVerticalIntakeMotorDutyCycle = 0.7 #0.8
kHorizontalIntakeMotorDutyCycle = 0.9
kHorizontalIntakeMotorDutyCycleRaisedAuto = 0.9
kVerticalIntakeMotorDutyCycleRaisedAuto = 0.9

kRedSpeakerPose = Pose2d(16.58, 5.547, Rotation2d())
kBlueSpeakerPose = Pose2d(-0.0381, 5.547, Rotation2d()) #5.547
