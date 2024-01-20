import constants
import wpilib
from commands2 import *
from commands2.button import * 
from commands2.cmd import * 

from subsystems.feeder import FeederSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem


class RobotContainer(object):
    
    def __init__(self) -> None:
        self.configure_button_bindings()

    def __init__(self):
        self.feeder = FeederSubsystem()
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()

        self.driver_controller = CommandXboxController(
            constants.kDriverControllerPort)
            
        
                            
        self.operator_controller = CommandXboxController(            
            constants.kOperatorControllerPort)
        
        self.configure_button_bindings()

    def configure_button_bindings(self):
        self.operator_controller.a().onTrue(self.shooter.shooter_on_cmd())
        self.operator_controller.b().onTrue(self.shooter.shooter_off_cmd())

        self.operator_controller.x().onTrue(self.feeder.feeder_on_cmd())
        self.operator_controller.y().onTrue(self.feeder.feeder_off_cmd())

        self.driver_controller.a().onTrue(self.intake.set_intake_cmd(0.5, 0.5)
                                          .until(self.intake.has_game_piece))
        self.operator_controller.rightBumper().onTrue(self.shooter.shooter_on_cmd().alongWith(self.feeder.feeder_on_cmd()
                                                                  .until(self.intake.has_game_piece))
                                                       .andThen(self.shooter.shooter_off_cmd())
                                                               .alongWith(self.feeder.feeder_off_cmd()))
            