import constants
import wpilib
from commands2 import *
from commands2.button import * 
from commands2.cmd import * 

from subsystems.feeder import FeederSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem
from subsystems.gripper import GripperSubsystem
from subsystems.arm import ArmSubsystem
from subsystems.drivetrain import DriveSubsystem
from subsystems.angulator import AngulatorSubsystem

from pathplannerlib.auto import NamedCommands
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder



class RobotContainer(object):

    def __init__(self):
        self.feeder = FeederSubsystem()
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()
        self.gripper = GripperSubsystem()
        self.arm = ArmSubsystem()
        self.drivetrain = DriveSubsystem()
        self.angulator = AngulatorSubsystem()

        self.driver_controller = CommandXboxController(
            constants.kDriverControllerPort)

        self.operator_controller = CommandXboxController(            
            constants.kOperatorControllerPort)
        
        self.button_bindings_configure()

        self.drivetrain.setDefaultCommand(self.drivetrain.drive_with_joystick_cmd(self.driver_controller))

    def button_bindings_configure(self):
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        self.operator_controller.a().onTrue(self.shooter.shooter_on_cmd(constants.kShooterSpeedRPS))
        self.operator_controller.b().onTrue(self.shooter.shooter_off_cmd())
        self.operator_controller.x().onTrue(self.arm.arm_extend_cmd())
        self.operator_controller.rightBumper().onTrue(self.amp_score_cmd())
        self.operator_controller.leftBumper().onTrue(self.amp_handoff_cmd())
        self.operator_controller.povUp().onTrue(self.angulator.angulator_up_cmd())
       # self.operator_controller.povUp().onFalse(self.angulator.angulator_off_cmd())
        self.operator_controller.povDown().onTrue(self.angulator.angulator_down_cmd())
       # self.operator_controller.povDown().onFalse(self.angulator.angulator_off_cmd())

        self.driver_controller.x().onTrue(self.feeder.feeder_on_cmd(constants.kFeederSpeedRPS))
        self.driver_controller.y().onTrue(self.feeder.feeder_off_cmd())
        self.driver_controller.a().onTrue(self.intake.set_intake_cmd(0.5, 0.5)
                                            .andThen(WaitUntilCommand(self.intake.has_game_piece))
                                            .andThen(self.intake.set_intake_cmd(0.0, 0.0)))
        self.driver_controller.rightBumper().onTrue(self.speaker_score_cmd())
        
        # self.driver_controller.leftBumper().onTrue(self.gripper.open_cmd())
        # self.driver_controller.rightBumper().onTrue(self.speaker_score_cmd())
            
    def speaker_score_cmd(self):
        return (self.feeder.feeder_on_cmd(constants.kFeederSpeedRPS)
                .andThen(WaitUntilCommand(self.feeder.feeder_piece_ejected))
                .andThen(self.feeder.feeder_off_cmd())
                .andThen(self.shooter.shooter_off_cmd()))
                
    
    def amp_handoff_cmd(self):
        return (self.shooter.shooter_to_arm_cmd().andThen(self.feeder.feeder_on_cmd)
                .andThen(WaitUntilCommand(self.feeder.feeder_piece_ejected))
                .andThen(self.gripper.gripper_close_cmd())
                .andThen(self.shooter.shooter_off_cmd())
                .andThen(self.feeder.feeder_off_cmd()))
    
    def amp_score_cmd(self):
        return (self.gripper.gripper_open_cmd()
        .andThen(self.arm.arm_retract_cmd))
    
                
    
    def getAutonomousCommand():
    # Load the path you want to follow using its name in the GUI
        path = PathPlannerPath.fromPathFile('ExampleWow')

        return PathPlannerAuto('Bottom Auto')

    # Create a path following command using AutoBuilder. This will also trigger event markers.
    #return AutoBuilder.followPathWithEvents(path);