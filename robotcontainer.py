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
from subsystems.climber import ClimberSubsystem

from subsystems.arm import ArmSubsystem
from subsystems.drivetrain import DriveSubsystem
from subsystems.angulator import AngulatorSubsystem

from pathplannerlib.auto import NamedCommands
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.auto import PathPlannerAuto

from commands2 import sysid
from wpilib.sysid import SysIdRoutineLog


class RobotContainer(object):

    def __init__(self):
        self.feeder = FeederSubsystem()
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()
        self.arm = ArmSubsystem()
        self.drivetrain = DriveSubsystem()
        self.angulator = AngulatorSubsystem()
        self.climber = ClimberSubsystem()
        
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
        NamedCommands.registerCommand('feeder on', self.feeder.feeder_on_cmd())
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
        self.operator_controller.rightTrigger().onTrue(self.amp_score_cmd())
        #self.operator_controller.leftTrigger().onTrue(self.prepare_to_score_amp_cmd())
        self.operator_controller.leftBumper().onTrue(self.shooter.shooter_off_cmd())
        self.operator_controller.povUp().onTrue(self.climber.kill_the_beast_cmd())
        self.operator_controller.povDown().onTrue(self.climber.Batman_grapling_hook_cmd())
        #self.operator_controller.back().onTrue(self.prepare_to_climb_cmd())
       
               
          
        self.driver_controller.a().onTrue(self.drivetrain.field_centric_cmd())
        self.driver_controller.b().onTrue(self.drivetrain.robot_centric_cmd())
        #self.driver_controller.x().onTrue(self.drivetrain.odometry reset)
        self.driver_controller.rightBumper().whileTrue(self.intake.set_intake_cmd(-0.5, -0.5))        
        self.driver_controller.rightBumper().onFalse(self.intake.set_intake_cmd(0.0, 0.0))
        self.driver_controller.rightTrigger().onTrue(self.intake.set_intake_cmd(0.9, 0.9))        
        #self.driver_controller.leftTrigger().onTrue(auto align)
        self.driver_controller.leftBumper().onTrue(self.speaker_score_cmd())
       
        
        #self.driver_controller.leftBumper().whileTrue(self.drivetrain.limelight_angulor_alignment_cmd(self.driver_controller))
        #self.driver_controller.povRight().whileTrue(self.drivetrain.line_up_with_april_tag_cmd(self.driver_controller))
        #self.driver_controller.rightBumper().whileTrue(self.drivetrain.drive_with_joystick_limelight_target_align_cmd(self.driver_controller))    
        
        #Use for sysID Test
        # self.diag_controller.povLeft().whileTrue(self.drivetrain.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        # self.diag_controller.povRight().whileTrue(self.drivetrain.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kForward))
        # self.diag_controller.povUp().whileTrue(self.drivetrain.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kForward))
        # self.diag_controller.povDown().whileTrue(self.drivetrain.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        
        self.diag_controller.povLeft().whileTrue(self.angulator.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        self.diag_controller.povRight().whileTrue(self.angulator.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kForward))
        self.diag_controller.povUp().whileTrue(self.angulator.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kForward))
        self.diag_controller.povDown().whileTrue(self.angulator.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        
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
        SmartDashboard.putData("Feeder On",self.feeder.feeder_on_cmd())
        SmartDashboard.putData("Zero Angulator", self.angulator.zero_angulator_encoder_cmd().ignoringDisable(True))
        
        SmartDashboard.putData("Amp Handoff", self.amp_handoff_cmd())
        

    def prepare_happy_donut_cmd(self):
        return (# angulator to happy donut position 
                self.angulator.angulator_set_pos_cmd(constants.kAngulatorHappyDonutAngleDeg))
                # shooter on to happy donut persentage
                #.alongWith(self.shooter.shooter_on_cmd(constants.kShooterSpeedHappyDonutRPS)))
    
    def prepare_subwoofer_shot_cmd(self):
        return(# angulator to subwoofer position 
               self.angulator.angulator_set_pos_cmd(constants.kAngulatorSubwooferAngleDeg))
               # shooter on to subwoofer persentage
               #.alongWith(self.shooter.shooter_on_cmd(constants.kShooterSpeedSubwooferRPS)))
        
    def prepare_speaker_shot_cmd(self):
        return (self.angulator.angulator_set_pos_from_range_cmd(self.drivetrain.get_range_to_speaker())
                .andThen(self.shooter.shooter_range_set_speed_cmd(self.drivetrain.get_range_to_speaker())))
                                   
    def speaker_score_cmd(self):
        return (self.arm.arm_stow_pos_cmd()
                .andThen(self.feeder.feeder_on_cmd())
                .andThen(WaitUntilCommand(self.feeder.feeder_piece_ejected))
                .andThen(WaitCommand(1))
                .andThen(self.feeder.feeder_off_cmd())
                .andThen(self.shooter.shooter_off_cmd()))
                #TODO angulator down               
    
    def amp_handoff_cmd(self):
        return (sequence(
                self.angulator.angulator_set_pos_cmd(0),
                self.arm.arm_pickup_pos_cmd(),
                self.angulator.angulator_amp_handoff_cmd().withTimeout(1.0),
                self.arm.arm_pickup_pos_cmd(12.0)))
                
                # self.shooter.shooter_to_arm_cmd(),
                # # self.feeder.feeder_on_cmd()
                # .andThen(WaitUntilCommand(self.feeder.feeder_piece_ejected))
                # .andThen(self.arm.gripper_close_cmd())
                # .andThen(self.shooter.shooter_off_cmd())
                # .andThen(self.feeder.feeder_off_cmd()))
    
    def amp_score_cmd(self):
        return (self.arm.gripper_open_cmd()
        .andThen(self.arm.arm_pickup_pos_cmd))
        
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