import constants
import commands2
import commands2.button

from subsystems.feeder import FeederSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem


class RobotContainer(object):
    
    def __init__(self) -> None:
        self.configure_button_bindings()

    def configure_button_bindings(self):
        pass

    def __init__(self):
        self.feeder = FeederSubsystem
        self.intake = IntakeSubsystem
        self.shooter = ShooterSubsystem

        self.driver_controller = commands2.button.CommandXboxController(
            constants.kDriverControllerPort)
        
        self.configure_button_bindings()

    def configure_button_bindings(self):
        self.driver_controller.A().onTrue(self.set_intake_cmd(0.5, 0.5))
        self.driver_controller.B().onTrue(self.set_intake_cmd(0, 0))
               
        self.operator_controller = commands2.button.CommandXboxController(            
            constants.kOperatorControllerPort)
        
        self.configure_button_bindings()

    def configure_button_bindings(self):
        self.operator_controller.A().onTrue(self.shooter_on_cmd)
        self.operator_controller.B().onTrue(self.shooter_off_cmd)

        self.operator_controller.x().onTrue(self.feeder_on_cmd)
        self.operator_controller.y().onTrue(self.feeder_off_cmd)9

    