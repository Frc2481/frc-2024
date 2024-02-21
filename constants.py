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
kAngulatorMotorCANID = 13
kArmMotorCANID = 14
kSwerveFrontLeftSteerEncoderCANID = 2
kSwerveFrontRightSteerEncoderCANID = 4
kSwerveBackLeftSteerEncoderCANID = 6
kSwerveBackrightSteerEncoderCANID = 8
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
kdriveV = 2.0825 * wpimath.units.inchesToMeters(kDistanceTraveledOneRotation)
#kdriveA = 0.24925
kdriveA = 0.24925 * wpimath.units.inchesToMeters(kDistanceTraveledOneRotation)
kdriveS = 0.04225
kDriveGearReduction = 125



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

#Feeder
kFeederSpeedRPS = 20
kFeederP = 5
kFeederI = 1
kFeederD = 0
kFeederV = 1.0

#BeamBrake
kIntakeBeambreakPort = 0
kFeederBeambreakPort = 1

#ShooterFocCurrentGains
kShooterSpeedRPS = 50
kShooterToArmSpeedRPS = 10
kShooterP = 0
kShooterI = 0
kShooterD = 0
kShooterV = 0
kShooterPeakCurrent = 40
kShooterCruiseVelocity = 0
kShooterAcceleration = 0
kShooterJerk = 0 

#Gripper
kGripperSolenoidModule = 0
kGripperDoubleSolenoidForwardPort = 1
kGripperDoubleSolenoidReversePort = 2



#Arm
kArmSolenoidModule = 0
kArmDoubleSolenoidForwardPort = 3
kArmDoubleSolenoidReversePort = 4

kArmSolenoid2Module = 0
kArmDoubleSolenoid2ForwardPort = 5
kArmDoubleSolenoid2ReversePort = 6

#Arm Motor
kArmP = 1
kArmI = 0.0
kArmD = 0
kArmV = 0.133
kArmA = 0.0
kArmS = 0.0
kArmAcceleration =800
kArmCruiseVelocity = 90


# FIND ARM POSITION
kArmUpPosition = 35.44
kArmStowPosition = 15.716
kArmDownPosition = 0



#angulator
kAngulatorGearReduction = 125.0
kAngulatorDownPosition = 0   
kAngulatorUpPosition = 47.5 
kAngulatorP = 2
kAngulatorI = 0# .01
kAngulatorD = 0.1
kAngulatorV = 0.02
kAngulatorA = 0 # 0.0025
kAngulatorS = 0
kAngulatorJerk = 0
kAngulatorAcceleration = 160
kAngulatorCruiseVelocity = 3 * 4096
kMotorResistance = 6
kAngulatorForwardSoftLimit = 11946
kAngulatorReverseSoftLimit = 0

#Metric system units
kArmMass = 10 / 2.205
kArmLength = 16 / 39.37 

#steer
kSteerGearReduction = 4

# https://prod.liveshare.vsengsaas.visualstudio.com/join?A43088F23A8DB725BF103347DA2FBF3ED7C8