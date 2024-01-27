import math

kSwerveFrontLeftDriveMotorCANID = 1
kSwerveFrontLeftSteerMotorCANID = 2
kSwerveFrontRightDriveMotorCANID = 3
kSwerveFrontRightSteerMotorCANID = 4
kSwerveBackLeftDriveMotorCANID =5
kSwerveBackLeftSteerMotorCANID = 6
kSwerveBackRightDriveMotorCANID = 7
kSwerveBackRightSteerMotorCANID = 8
kShooterMotorCANID = 9
kIntakeVerticalMotorCANID = 10
kIntakeHorizontalMotorCANID = 11
kFeederMotorCANID = 12
kAngulatorMotorCANID = 13
kSwerveFrontLeftSteerEncoderCANID = 2
kSwerveFrontRightSteerEncoderCANID = 4
kSwerveBackLeftSteerEncoderCANID = 6
kSwerveBackrightSteerEncoderCANID = 8
kPigeonCANID = 1

#drivetrain
kWheelTrack = 18.5
kWheelBase = 16.5
kdriveP = 0
kdriveI = 0
kdriveD = 0
kdriveV = 0

#steer
ksteerP = 0
ksteerI = 0
ksteerD = 0
ksteerV = 0

kSwerveReductionSteer = 40
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
kShooterP = 5
kShooterI = 1
kShooterD = 0
kShooterV = 1.0
kShooterPeakCurrent = 40

#Gripper
kGripperSolenoidModule = 0
kGripperDoubleSolenoidForwardPort = 1
kGripperDoubleSolenoidReversePort = 2


#Arm
kArmSolenoidModule = 0
kArmDoubleSolenoidForwardPort = 3
kArmDoubleSolenoidReversePort = 4

kDriveMaxSpeed = 16
kDrivePeriod = 100

#wheeelly wheel
kDriveWheelRadiusIn = 1.5
kDistanceTraveledOneRotation = (kDriveWheelRadiusIn * 2 * math.pi) 
kDriveBaseRadiusIn = (((kWheelTrack**2 + kWheelBase**2)**0.5)/2)

kMaxModuleSpeedFt = 19

kDrivePathTranslationKP = 5.0
kDrivePathTranslationKI = 0.0
kDrivePathTranslationKD = 0.0

kDrivePathAngularKP = 5.0
kDrivePathAngularKI = 0.0
kDrivePathAngularKD = 0.0

#angulator
kAngulatorGearReduction = 125.0
kAngulatorDownPosition = 25  
kAngulatorUpPosition = 70
kAngulatorP = 5
kAngulatorI = 0
kAngulatorD = 0
kAngulatorV = 0.12 
kAngulatorA = 0.025
kAngulatorS = 0
kAngulatorJerk = 0
kAngulatorAcceleration = 160
kAngulatorCruiseVelocity = 80

kMotorResistance = 6

#Metric system units
kArmMass = 10 / 2.205
kArmLength = 16 / 39.37 

