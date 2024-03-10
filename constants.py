import math
import wpimath.units

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

kDriveMaxSpeed = 6.4 
kDrivePeriod = 20

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
kdriveV = 2.0496 * wpimath.units.inchesToMeters(kDistanceTraveledOneRotation)
#kdriveA = 0.24925
kdriveA = 0.077519 * wpimath.units.inchesToMeters(kDistanceTraveledOneRotation)
kdriveS = 0.1891
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
kSwerveReductionDrive = 4.31

kDriverControllerPort = 0
kOperatorControllerPort = 1
kDiagControllerPort = 2

#Feeder
kFeederSpeedRPS = 10
kFeederP = 5
kFeederI = 1
kFeederD = 0
kFeederV = 1.0
kFeederSpeed = 0.2

#BeamBrake
kFeederBeambreakStageTwoPort = 9
kFeederBeambreakStageOnePort = 7
kArmZero = 8

#ShooterFocCurrentGains
kShooterSpeedSubwooferRPS = 65
kShooterSpeedHappyDonutRPS = 82
kShooterSpeedMaxRPS = 82
kShooterReverseSpeed = -10
kShooterToArmSpeedRPS = 10
kShooterP = 0.01
kShooterI = 0
kShooterD = 0
kShooterV = 0.0125
kShooterA = 0
kShooterS = 0.02
kShooterPeakCurrent = 40
kShooterCruiseVelocity = 0
kShooterAcceleration = 0
kShooterJerk = 0
kShooterOnTarget = 1 

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
kArmScorePosition = 34.916504
kArmClimbPosition = 34
kArmPickupPosition = 13.4
kArmDownPosition = 1

#angulator
kAngulatorHappyDonutAngleDeg = 0.002
kAngulatorSubwooferAngleDeg = 31.64 / 360
kAngulatorGearReduction = 259.9
kAngulatorDownPosition = 0 # 20.5/360   
kAngulatorUpPosition =  0.112 #/360
kAngulatorP = 150
kAngulatorI = 0.02# .01
kAngulatorD = 0
kAngulatorV = 30.7 # 0.041
kAngulatorA = 0.00 # 0.0025
kAngulatorS = 0.3
kAngulatorJerk = 0
kAngulatorAcceleration = 2.0
kAngulatorCruiseVelocity = 0.31
kMotorResistance = 6
kAngulatorForwardSoftLimitRot = 0.13 # 70 / 360
kAngulatorReverseSoftLimitRot = 0.00488 # 22.5 / 360
kAngulatorOnTarget = 0.001

#Metric system units
kArmMass = 10 / 2.205
kArmLength = 16 / 39.37 

#steer
kSteerGearReduction = 4
