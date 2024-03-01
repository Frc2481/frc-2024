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
        self.beambreak = DigitalInput(constants.kFeederBeambreakPort)
        self.beambreak_trigger = Trigger(self.beambreak.get)
        self.beambreak_trigger.onFalse(self.beambreak_false_cmd())
        self.beambreak_trigger.onTrue(self.beambreak_true_cmd())
        self.align_state = constants.kAlignStateNone
        
        self.driver_controller = CommandXboxController(
            constants.kDriverControllerPort)

        self.operator_controller = CommandXboxController(            
            constants.kOperatorControllerPort)
        
        self.diag_controller = CommandXboxController(
            constants.kDiagControllerPort)
        
        NamedCommands.registerCommand('shooter on', self.shooter.shooter_on_cmd(constants.kShooterSpeedRPS))
        NamedCommands.registerCommand('shooter off', self.shooter.shooter_off_cmd())
        NamedCommands.registerCommand('arm up position', self.arm.arm_score_pos_cmd())
        NamedCommands.registerCommand('arm down position', self.arm.arm_stow_pos_cmd())
        NamedCommands.registerCommand('arm pick up position', self.arm.arm_pickup_pos_cmd())
        NamedCommands.registerCommand('gripper open', self.arm.gripper_open_cmd())
        NamedCommands.registerCommand('gripper close', self.arm.gripper_close_cmd())
        NamedCommands.registerCommand('feeder on', self.feeder.feeder_on_cmd(.9))
        NamedCommands.registerCommand('feeder off',self.feeder.feeder_off_cmd())
        NamedCommands.registerCommand('intake on', self.intake.set_intake_cmd(0.9, 0.9))
        NamedCommands.registerCommand('intake off', self.intake.set_intake_cmd(0, 0))
        
        self.button_bindings_configure()

        self.drivetrain.setDefaultCommand(self.drivetrain.drive_with_joystick_cmd(self.driver_controller))
        DataLogManager.start()

    def button_bindings_configure(self):
        print("Here 3")
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        self.operator_controller.a().onTrue(self.prepare_happy_donut_cmd())
        self.operator_controller.x().onTrue(self.prepare_subwoofer_shot_cmd())
        self.operator_controller.rightBumper().onTrue(self.prepare_speaker_shot_cmd())
        self.operator_controller.rightTrigger().onTrue(self.score_amp_stow_arm_cmd())
        self.operator_controller.leftTrigger().onTrue(self.amp_handoff_cmd())
        self.operator_controller.leftBumper().onTrue(self.shooter.shooter_off_cmd())
        self.operator_controller.povUp().onTrue(self.arm.kill_the_beast_cmd())
        self.operator_controller.povDown().onTrue(self.arm.batman_grappling_hook_cmd())
        self.operator_controller.back().onTrue(self.prepare_to_climb_cmd())
       
                         
        self.driver_controller.a().onTrue(self.drivetrain.field_centric_cmd())
        self.driver_controller.b().onTrue(self.drivetrain.robot_centric_cmd())
        self.driver_controller.x().onTrue(self.drivetrain.reset_yaw_cmd())
        self.driver_controller.rightBumper().whileTrue(self.intake_sequence_cmd(-0.5, -0.5, -0.5))        
        self.driver_controller.rightBumper().onFalse(self.intake_sequence_cmd(0.0, 0.0, 0.0))
        self.driver_controller.rightTrigger().onTrue(self.intake_feeder_cmd(constants.kFeederSpeed, 0.9, 0.9))        
        self.driver_controller.leftTrigger().whileTrue(self.drivetrain.limelight_align_cmd(self.driver_controller, self.get_align_state))
        # self.driver_controller.leftTrigger().onFalse(self.set_align_state_cmd(constants.kAlignStateNone))
        self.driver_controller.leftBumper().onTrue(self.speaker_score_cmd())
       
        
        #self.driver_controller.leftBumper().whileTrue(self.drivetrain.limelight_angulor_alignment_cmd(self.driver_controller))
        #self.driver_controller.povRight().whileTrue(self.drivetrain.line_up_with_april_tag_cmd(self.driver_controller))
        #self.driver_controller.rightBumper().whileTrue(self.drivetrain.drive_with_joystick_limelight_target_align_cmd(self.driver_controller))    
        
        #Use for sysID Test
        self.diag_controller.povLeft().whileTrue(self.drivetrain.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        self.diag_controller.povRight().whileTrue(self.drivetrain.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kForward))
        self.diag_controller.povUp().whileTrue(self.drivetrain.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kForward))
        self.diag_controller.povDown().whileTrue(self.drivetrain.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        
        # self.diag_controller.povLeft().whileTrue(self.angulator.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        # self.diag_controller.povRight().whileTrue(self.angulator.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kForward))
        # self.diag_controller.povUp().whileTrue(self.angulator.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kForward))
        # self.diag_controller.povDown().whileTrue(self.angulator.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        
        self.diag_controller.a().onTrue(self.angulator.zero_angulator_encoder_cmd())
        
        SmartDashboard.putData("Reset Odom", InstantCommand(lambda: self.drivetrain.reset_pose()).ignoringDisable(True))
        SmartDashboard.putData("Zero Steer Encoder", self.drivetrain.zero_steer_encoder_cmd())
        SmartDashboard.putData("Calibrate Wheel Circumference", self.drivetrain.calibrate_wheel_circumference_cmd())
        SmartDashboard.putData("Reset Odom To Vision", self.drivetrain.reset_odom_to_vision_cmd().ignoringDisable(True))
        
        SmartDashboard.putData("Arm Up", self.arm.arm_score_pos_cmd())
        SmartDashboard.putData("Arm Down", self.arm.arm_stow_pos_cmd())
        SmartDashboard.putData("Arm Pick Up", self.arm.arm_pickup_pos_cmd())
        SmartDashboard.putData("Arm Climb", self.arm.arm_climb_pos_cmd())
        
        SmartDashboard.putData("Gripper Open", self.arm.gripper_open_cmd())
        SmartDashboard.putData("Gripper Close", self.arm.gripper_close_cmd())
        
        SmartDashboard.putData("Shooter On", self.shooter.shooter_on_cmd())
        SmartDashboard.putData("Shooter Off", self.shooter.shooter_off_cmd())
        SmartDashboard.putData("Feeder Off",self.feeder.feeder_off_cmd())
        SmartDashboard.putData("Feeder On",self.feeder.feeder_on_cmd(.9))
                
        SmartDashboard.putData("Zero Angulator", self.angulator.zero_angulator_encoder_cmd().ignoringDisable(True))
        
        SmartDashboard.putData("Amp Handoff", self.amp_handoff_cmd())
        
        SmartDashboard.putData("Reset Arm and Angulator", self.score_amp_stow_arm_cmd ())
        
        SmartDashboard.putData("Intake On", self.intake_feeder_cmd(constants.kFeederSpeed, 0.9, 0.9))
        SmartDashboard.putData("Intake Off", self.intake_feeder_off_cmd())
        
        SmartDashboard.putNumber("Shooter Voltage", self.shooter.shooterMotor.get_motor_voltage().value)
        
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
        
    def prepare_speaker_shot_cmd(self):
        return (sequence(
                self.set_align_state_cmd(constants.kAlignStateSpeaker),
                self.angulator.angulator_set_pos_from_range_cmd(self.drivetrain.get_range_to_speaker),
                self.shooter.shooter_on_cmd(constants.kShooterSpeedHappyDonutRPS))
                # self.shooter.shooter_range_set_speed_cmd(self.drivetrain.get_range_to_speaker()))
            )
                                   
    def speaker_score_cmd(self):
        return (sequence(
                self.feeder.feeder_on_cmd(.9),
                WaitCommand(2),
                self.feeder.feeder_off_cmd(),
                self.shooter.shooter_off_cmd(),
                self.angulator.angulator_set_pos_cmd(0)))               
    
    def amp_handoff_cmd(self):
        return (sequence(
                self.set_align_state_cmd(constants.kAlignStateAmp),
                self.angulator.angulator_set_pos_cmd(0),
                self.arm.arm_pickup_pos_cmd(11.5),
                self.angulator.angulator_amp_handoff_cmd().withTimeout(1.0),
                self.shooter.shooter_on_cmd(25),
                self.feeder.feeder_on_cmd(.5),
                self.arm.gripper_close_cmd(),
                WaitCommand(.1),
                self.arm.arm_score_pos_cmd(),
                WaitCommand(.2),
                self.shooter.shooter_off_cmd(),
                self.feeder.feeder_off_cmd(),
                WaitCommand(0.5),
                self.angulator.angulator_set_pos_cmd(0)))
                
    def prepare_to_climb_cmd(self):
        return (sequence(
            self.arm.arm_climb_pos_cmd(),
            self.arm.gripper_open_cmd()))
                
    def score_amp_stow_arm_cmd (self):
        return(sequence(
            self.arm.gripper_open_cmd(),
            self.angulator.angulator_set_pos_cmd(0),
            self.arm.arm_stow_pos_cmd()))                
        
    #def unimportant_sim_stuff_cmd(self):
        #return (InstantCommand(lambda: self.drivetrain._fl.driveMotor.sim_state.set_raw_rotor_position(8.25))
        #.andThen(InstantCommand(lambda: self.drivetrain._fr.driveMotor.sim_state.set_raw_rotor_position(8.25)))
       # .andThen(InstantCommand(lambda: self.drivetrain._bl.driveMotor.sim_state.set_raw_rotor_position(8.25)))
        #.andThen(InstantCommand(lambda: self.drivetrain._br.driveMotor.sim_state.set_raw_rotor_position(8.25)))
        #.andThen(InstantCommand(lambda: self.drivetrain._gyro.sim_state.set_raw_yaw(359))))  
                
    def getAutonomousCommand(self):
    # Load the path you want to follow using its name in the GUI
        # self._sd = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")
        # self._sd.put("Run Auto", )
        
        # SmartDashboard.putData(AutoBuilder.bui)
    
        # path = PathPlannerPath.fromPathFile('Sample for programmers')
        return PathPlannerAuto("Programmers Auto")
    
        return AutoBuilder.followPath(path)
    
    def intake_sequence_cmd(self, feeder_cmd, horizontal, vertical):
        return sequence(self.score_amp_stow_arm_cmd(),
                        self.intake.set_intake_cmd(horizontal, vertical),
                        self.feeder.feeder_on_cmd(feeder_cmd))
    
    def intake_feeder_cmd(self, feeder_cmd, intake_cmd_horizontal, intake_cmd_vertical):
        return(sequence(
            self.set_align_state_cmd(constants.kAlignStateNote),
            self.intake_sequence_cmd(feeder_cmd, intake_cmd_horizontal, intake_cmd_vertical))
            )
    
    def intake_feeder_off_cmd(self):
        return(self.intake.intake_off_cmd()) \
        .alongWith(self.feeder.feeder_off_cmd())\
        .andThen(self.shooter.shooter_on_cmd(constants.kShooterReverseSpeed))\
        .andThen(WaitCommand(.1))\
        .andThen(self.shooter.shooter_off_cmd())
    
    
    def beambreak_false_callback(self):
        SmartDashboard.putNumber("Feeder BeamBreak", False)
        self.intake.horizontalMotor.set_control(VoltageOut(0))
        self.feeder.feederMotor.set_control(VoltageOut(0))
        self.intake.verticalMotor.set_control(VoltageOut(0))
    
    def beambreak_true_callback(self):
        SmartDashboard.putNumber("Feeder BeamBreak", True)
        PrintCommand("DGT: beambreak onTrue") 
        
    def beambreak_false_cmd(self):
        return sequence(
             InstantCommand(lambda: self.intake.horizontalMotor.set_control(VoltageOut(0))),
             InstantCommand(lambda: self.intake.verticalMotor.set_control(VoltageOut(0))),
          
             self.shooter.shooter_on_cmd(-1),
             self.feeder.feeder_on_cmd(-.05),
             WaitUntilCommand(lambda: self.beambreak.get() == True),
             self.shooter.shooter_off_cmd(),
             self.feeder.feeder_off_cmd()
        )
    
    def beambreak_true_cmd(self):
        return (runOnce(lambda: self.beambreak_true_callback()))
    
    def periodic(self):
        SmartDashboard.putNumber("Align State", self.align_state)
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance())
    
    