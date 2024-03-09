import constants
import wpilib
from wpilib import SmartDashboard, DataLogManager
from commands2 import *
from commands2.button import * 
from commands2.cmd import * 
import ntcore

from subsystems.feeder import FeederSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem

from subsystems.arm import ArmSubsystem
from subsystems.drivetrain import DriveSubsystem
from subsystems.angulator import AngulatorSubsystem

from pathplannerlib.auto import NamedCommands
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.auto import PathPlannerAuto

from ntcore import NetworkTableInstance

from commands2 import sysid
from wpilib.sysid import SysIdRoutineLog
from wpilib import DigitalInput
from phoenix6.controls import VoltageOut

class RobotContainer(Subsystem):

    def __init__(self):
        self.feeder = FeederSubsystem()
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()
        self.arm = ArmSubsystem()
        self.drivetrain = DriveSubsystem()
        self.angulator = AngulatorSubsystem()    
        self.auto_mode = False 
        
        NamedCommands.registerCommand('prepare speaker shot', self.prepare_auto_speaker_shot_cmd())
        NamedCommands.registerCommand('intake feeder on', self.intake_feeder_cmd(constants.kFeederSpeed, 0.9, 0.3))
        NamedCommands.registerCommand('speaker score', self.speaker_score_cmd())
        NamedCommands.registerCommand('prepare first amp shot', self.prep_first_amp_shot_auto())
        NamedCommands.registerCommand('prepare first feeder shot', self.prep_first_feeder_shot_auto())
        NamedCommands.registerCommand('shooter off', self.shooter.shooter_off_cmd())
        NamedCommands.registerCommand('wait command', WaitCommand(0.25))
        
        self.beambreak_one = DigitalInput(constants.kFeederBeambreakStageOnePort)
        self.beambreak_trigger_one = Trigger(self.beambreak_one.get)
        self.beambreak_trigger_one.onFalse(self.beambreak_one_false_cmd())

        self.beambreak_two = DigitalInput(constants.kFeederBeambreakStageTwoPort)
        self.beambreak_trigger_two = Trigger(self.beambreak_two.get)
        self.beambreak_trigger_two.onFalse(self.beambreak_two_false_cmd())
        
        self.align_state = constants.kAlignStateSpeaker       
         
        self.driver_controller = CommandXboxController(
            constants.kDriverControllerPort)
        self.operator_controller = CommandXboxController(            
            constants.kOperatorControllerPort)
    
        self.button_bindings_configure()
        self.drivetrain.setDefaultCommand(self.drivetrain.drive_with_joystick_cmd(self.driver_controller))
        
        SmartDashboard.putNumber("shooter_cmd_angle", 0)
        SmartDashboard.putNumber("shooter_cmd_speed", 0)
        # DataLogManager.start()


    def button_bindings_configure(self):
        print("Here 3")
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        self.operator_controller.a().onTrue(self.prepare_happy_donut_cmd())
        self.operator_controller.x().onTrue(self.prepare_subwoofer_shot_cmd())
        self.operator_controller.rightBumper().whileTrue(self.prepare_speaker_shot_cmd())
        self.operator_controller.rightTrigger().onTrue(self.score_amp_stow_arm_cmd())
        self.operator_controller.leftTrigger().onTrue(self.amp_handoff_cmd())
        self.operator_controller.leftBumper().onTrue(self.shooter.shooter_off_cmd())
        self.operator_controller.povUp().onTrue(self.arm.kill_the_beast_cmd())
        self.operator_controller.povDown().onTrue(self.arm.batman_grappling_hook_cmd())
        self.operator_controller.back().onTrue(self.prepare_to_climb_cmd())
                               
        self.driver_controller.a().onTrue(self.drivetrain.field_centric_cmd())
        self.driver_controller.b().onTrue(self.drivetrain.robot_centric_cmd())
        self.driver_controller.x().onTrue(self.drivetrain.reset_yaw_cmd())
        self.driver_controller.rightBumper().whileTrue(self.vomit_cmd())        
        self.driver_controller.rightBumper().onFalse(self.intake_sequence_cmd(0.0, 0.0, 0.0).alongWith(self.shooter.shooter_off_cmd()))
        self.driver_controller.rightTrigger().onTrue(self.intake_feeder_cmd(constants.kFeederSpeed, 0.9, 0.8))        
        self.driver_controller.leftTrigger().whileTrue(self.drivetrain.limelight_align_cmd(self.driver_controller, self.get_align_state))
        self.driver_controller.leftBumper().onTrue(self.speaker_score_cmd())
        
        #Use for sysID Test
        self.driver_controller.povUp().onTrue(self.angulator.angulator_up_cmd())
        self.driver_controller.povDown().onTrue(self.angulator.angulator_down_cmd())

        SmartDashboard.putData("Reset Odom", InstantCommand(lambda: self.drivetrain.reset_pose()).ignoringDisable(True))
        SmartDashboard.putData("Zero Steer Encoder", self.drivetrain.zero_steer_encoder_cmd())
        # SmartDashboard.putData("Calibrate Wheel Circumference", self.drivetrain.calibrate_wheel_circumference_cmd())
        # SmartDashboard.putData("Reset Odom To Vision", self.drivetrain.reset_odom_to_vision_cmd().ignoringDisable(True))
        SmartDashboard.putData("+shoot speed", self.shooter.increase_shooter_speed_cmd()) 
        SmartDashboard.putData("-shoot speed", self.shooter.decrease_shooter_speed_cmd())       
        SmartDashboard.putData("Zero Angulator", self.angulator.zero_angulator_encoder_cmd().ignoringDisable(True))
        
        
    def get_align_state(self):
        return self.align_state


    def prepare_happy_donut_cmd(self):
        return (# angulator to happy donut position 
                self.angulator.angulator_set_pos_cmd(constants.kAngulatorHappyDonutAngleDeg)
                # shooter on to happy donut persentage
                .alongWith(self.shooter.shooter_on_cmd(constants.kShooterSpeedHappyDonutRPS)))


    def prepare_subwoofer_shot_cmd(self):
        return(# angulator to subwoofer position 
               self.angulator.angulator_set_pos_cmd(constants.kAngulatorSubwooferAngleDeg)
               # shooter on to subwoofer persentage
               .alongWith(self.shooter.shooter_on_cmd(constants.kShooterSpeedSubwooferRPS)))


    def set_align_state(self, state):
        self.align_state = state


    def set_align_state_cmd(self, state):
        return runOnce(lambda: self.set_align_state(state))


    def set_angle_speed_from_range(self): 
        self.angulator.set_pos_from_range(self.drivetrain.get_range_to_speaker)
        self.shooter.set_speed_from_range(self.drivetrain.get_range_to_speaker)
    

    def prepare_speaker_shot_cmd(self):
        return (
            FunctionalCommand(lambda: self.set_align_state(constants.kAlignStateSpeaker),
                              lambda: self.set_angle_speed_from_range(),
                              lambda interrupted: None,
                              lambda: self.driver_controller.leftBumper().getAsBoolean(),
                                self.angulator)
                )
    
    def prepare_auto_speaker_shot_cmd(self):
        return (
            FunctionalCommand(lambda: self.set_align_state(constants.kAlignStateSpeaker),
                              lambda: self.set_angle_speed_from_range(),
                              lambda interrupted: None,
                              lambda: self.shooter.shooterMotor.get_closed_loop_error().value < constants.kShooterOnTarget,
                                self.angulator)
                )



    def speaker_score_cmd(self):
            return (sequence(
                # self.angulator.wait_for_angulator_on_target(),
                self.shooter.wait_for_shooter_on_target(),
                self.feeder.feeder_on_cmd(.9),
                WaitCommand(0.25),
                self.feeder.feeder_off_cmd(),
                self.shooter.shooter_off_cmd(),
                self.angulator.angulator_set_pos_cmd(0)
        ))               


    def amp_handoff_cmd(self):
        return (sequence(
                self.set_align_state_cmd(constants.kAlignStateAmp),
                self.intake.set_intake_cmd(0, 0),
                self.feeder.feeder_on_cmd(0),
                self.shooter.shooter_on_cmd(20),
                self.angulator.angulator_set_pos_cmd(0),
                self.arm.arm_pickup_pos_cmd(11.5),
                self.angulator.angulator_amp_handoff_cmd().withTimeout(1.0),
                self.feeder.feeder_on_cmd(.9),
                self.arm.gripper_close_cmd(),
                WaitCommand(.1),
                self.arm.arm_score_pos_cmd().withTimeout(1.0),
                WaitCommand(0.5),
                self.shooter.shooter_off_cmd(),
                self.feeder.feeder_off_cmd(),
                self.angulator.angulator_set_pos_cmd(0)))


    def prepare_to_climb_cmd(self):
        return (sequence(
            self.arm.arm_climb_pos_cmd(),
            self.arm.gripper_open_cmd()))


    def score_amp_stow_arm_cmd(self):
        return(sequence(
            self.arm.gripper_open_cmd(),
            WaitCommand(.2),
            self.angulator.angulator_set_pos_cmd(0),
            self.arm.arm_stow_pos_cmd(),
            self.set_align_state_cmd(constants.kAlignStateSpeaker)))       


    def prep_first_amp_shot_auto(self):
        return(sequence(
            self.shooter.shooter_on_cmd(65),
            self.angulator.angulator_set_pos_cmd(0.014)))
            #0.014 works with fresh battery
    

    def prep_first_feeder_shot_auto(self):
        return(sequence(
            self.shooter.shooter_on_cmd(65),
            self.angulator.angulator_set_pos_cmd(0.011)))


    def getAutonomousCommand(self):
        return PathPlannerAuto("6 piece")

    
    def intake_sequence_cmd(self, feeder_cmd, horizontal, vertical):
        return sequence(self.angulator.angulator_set_pos_cmd(0),
                        self.arm.arm_stow_pos_cmd(),
                        self.intake.set_intake_cmd(horizontal, vertical),
                        self.feeder.feeder_on_cmd(feeder_cmd)
                        #WaitUntilCommand(lambda: self.beambreak_one.get() == False),
                        #ScheduleCommand(self.beambreak_during_intake_cmd())
                        )


    def vomit_cmd(self):
        return (sequence(
                self.shooter.shooter_on_cmd(-50),
                self.intake_sequence_cmd(-0.5, -0.5, -0.5)
                ))


    def intake_feeder_cmd(self, feeder_cmd, intake_cmd_horizontal, intake_cmd_vertical):
        return(sequence(
            self.set_align_state_cmd(constants.kAlignStateNote),
            self.intake_sequence_cmd(feeder_cmd, intake_cmd_horizontal, intake_cmd_vertical))
            )
    
        
    def beambreak_during_intake_cmd(self):
        return (
             InstantCommand(lambda: SmartDashboard.putNumber("beambreak one", False)).alongWith(
             InstantCommand(lambda: self.intake.horizontalMotor.set_control(VoltageOut(0)))).alongWith(
             self.feeder.feeder_on_cmd(0.15))
        )


    def beambreak_one_false_cmd(self):
        return self.beambreak_during_intake_cmd()

    
    def beambreak_two_false_cmd(self):
        return InstantCommand(lambda: self.feeder.feederMotor.set_control(VoltageOut(0))).alongWith(
                       InstantCommand(lambda: self.intake.horizontalMotor.set_control(VoltageOut(0)))).alongWith(
                       InstantCommand(lambda: self.intake.verticalMotor.set_control(VoltageOut(0))))


    def periodic(self):
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance())
        SmartDashboard.putNumber("speak range", self.drivetrain.get_range_to_speaker())
        SmartDashboard.putNumber("shoot speed", self.shooter.shooterMotor.get_velocity().value)
