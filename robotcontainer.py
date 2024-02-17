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
#from subsystems.angulator import AngulatorSubsystem

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
        #self.angulator = AngulatorSubsystem()

        self.driver_controller = CommandXboxController(
            constants.kDriverControllerPort)

        self.operator_controller = CommandXboxController(            
            constants.kOperatorControllerPort)
        
        self.button_bindings_configure()
        
        # CommandScheduler.getInstance().onCommandInitialize(lambda x: print("Execute", x))
        #CommandScheduler.getInstance().onCommandExecute(lambda x: print("Execute", x))
        #CommandScheduler.getInstance().onCommandFinish(lambda x: print("Finish", x))
        #CommandScheduler.getInstance().onCommandInterrupt(lambda x: print("Interrupt", x))

        self.drivetrain.setDefaultCommand(self.drivetrain.drive_with_joystick_cmd(self.driver_controller))
        DataLogManager.start()

    def button_bindings_configure(self):
        print("Here 3")
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        self.operator_controller.a().onTrue(self.shooter.shooter_on_cmd(constants.kShooterSpeedRPS))
        self.operator_controller.b().onTrue(self.shooter.shooter_off_cmd())
        self.operator_controller.y().whileTrue(self.shooter.shooter_on_cmd(-1 * constants.kShooterSpeedRPS))
        self.operator_controller.x().onTrue(self.arm.arm_up_cmd())
        self.operator_controller.start().onTrue(self.arm.arm_down_cmd())
        self.operator_controller.rightBumper().onTrue(self.arm.gripper_open_cmd())
        self.operator_controller.leftBumper().onTrue(self.arm.gripper_close_cmd())
        #self.operator_controller.povRight().onTrue(self.unimportant_sim_stuff_cmd())
        self.operator_controller.povUp().onTrue(self.arm.arm_pickup_position_cmd())
        
        
        #Use for sysID Test
        #self.operator_controller.povLeft().whileTrue(self.drivetrain.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        #self.operator_controller.povRight().whileTrue(self.drivetrain.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kForward))
        #self.operator_controller.povUp().whileTrue(self.drivetrain.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kForward))
        #self.operator_controller.povDown().whileTrue(self.drivetrain.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        
        self.driver_controller.x().onTrue(self.feeder.feeder_on_cmd(constants.kFeederSpeedRPS))
        self.driver_controller.y().onTrue(self.feeder.feeder_off_cmd())
        self.driver_controller.a().onTrue(self.intake.set_intake_cmd(0.9, 0.9))
                                           # .andThen(WaitUntilCommand(self.intake.has_game_piece))
                                           # .andThen(self.intake.set_intake_cmd(0.0, 0.0)))
        self.driver_controller.rightBumper().onTrue(self.speaker_score_cmd())
        self.driver_controller.leftBumper().whileTrue(self.drivetrain.drive_with_joystick_limelight_align_cmd(self.driver_controller))
        self.driver_controller.povRight().whileTrue(self.drivetrain.line_up_with_joystick_limelight_align_cmd(self.driver_controller))
        self.driver_controller.povLeft().whileTrue(self.intake.set_intake_cmd(-0.5, -0.5))
        self.driver_controller.b().whileTrue(self.feeder.feeder_on_cmd(-1 * constants.kFeederSpeedRPS))
        self.driver_controller.povDown().onTrue(self.intake.set_intake_cmd(0.0, 0.0))
        
        
        SmartDashboard.putData("Reset Odom", InstantCommand(lambda: self.drivetrain.reset_pose()).ignoringDisable(True))
        SmartDashboard.putData("Zero Steer Encoder", self.drivetrain.zero_steer_encoder_cmd())
        #SmartDashboard.putData("Angulator Up", InstantCommand(lambda: self.angulator.angulator_up_cmd(False)))
        #SmartDashboard.putData("Angulator Down", InstantCommand(lambda: self.angulator.angulator_down_cmd(False)))
        SmartDashboard.putData("Calibrate Wheel Circumference", self.drivetrain.calibrate_wheel_circumference_cmd().ignoringDisable(True))
        

                        
       
    def speaker_score_cmd(self):
        return (self.feeder.feeder_on_cmd(constants.kFeederSpeedRPS)
                .andThen(WaitUntilCommand(self.feeder.feeder_piece_ejected))
                .andThen(self.feeder.feeder_off_cmd())
                .andThen(self.shooter.shooter_off_cmd()))
                
    
    def amp_handoff_cmd(self):
        return (self.shooter.shooter_to_arm_cmd().andThen(self.feeder.feeder_on_cmd)
                .andThen(WaitUntilCommand(self.feeder.feeder_piece_ejected))
                .andThen(self.arm.gripper_close_cmd())
                .andThen(self.shooter.shooter_off_cmd())
                .andThen(self.feeder.feeder_off_cmd()))
    
    def amp_score_cmd(self):
        return (self.arm.gripper_open_cmd()
        .andThen(self.arm.arm_pickup_position_cmd))
        
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
        return PathPlannerAuto("Gluke's Bottom Auto pt 2")
    
        return AutoBuilder.followPath(path)