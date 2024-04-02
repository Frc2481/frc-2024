import constants
import wpilib
from wpilib import SmartDashboard, DataLogManager, DriverStation, SendableChooser
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
from phoenix6.controls import VoltageOut, VelocityDutyCycle
from phoenix5.led import CANdle
from commands2 import ConditionalCommand

from phoenix6.signal_logger import SignalLogger

class RobotContainer(Subsystem):

    def __init__(self):
        self.feeder = FeederSubsystem()
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()
        self.arm = ArmSubsystem()
        self.drivetrain = DriveSubsystem()
        self.angulator = AngulatorSubsystem()    
        self.auto_mode = False 
        
        self.IgnoreBeamBreaks = False
        
        NamedCommands.registerCommand('prepare speaker shot', self.prepare_auto_speaker_shot_cmd())
        NamedCommands.registerCommand('prepare first amp shot', self.prepare_auto_shooter_and_angulator_cmd(65, 0.010))
        NamedCommands.registerCommand('prepare second amp speaker shot', self.prepare_auto_shooter_and_angulator_cmd(82, 0.004))
        NamedCommands.registerCommand('prepare third amp speaker shot', self.prepare_auto_shooter_and_angulator_cmd(65, 0.08))
        NamedCommands.registerCommand('speaker score', self.speaker_score_cmd())        
        NamedCommands.registerCommand('auto speaker score', self.auto_shoot_cmd())
        NamedCommands.registerCommand('no wait speaker score', self.no_wait_auto_shoot_cmd()) 
        
        NamedCommands.registerCommand('prepare source auto first shot', self.auto_prep_shoot_cmd(80, 0.013))
        NamedCommands.registerCommand('prepare source auto second shot', self.auto_prep_fast(76, 0.018))
        NamedCommands.registerCommand('prepare source auto third shot', self.auto_prep_fast(76, 0.02))
        NamedCommands.registerCommand('prepare source auto fourth shot', self.auto_prep_fast(76, 0.02))
        
        NamedCommands.registerCommand('prepare slow front auto first shot', self.auto_prep_shoot_cmd(65, 0.06))
        NamedCommands.registerCommand('prepare slow front auto second shot', self.auto_prep_fast_raised(76, 0.036))
        NamedCommands.registerCommand('prepare slow front auto third shot', self.auto_prep_fast_raised(76, 0.041))
        NamedCommands.registerCommand('prepare slow front auto fourth shot', self.auto_prep_fast_raised(76, 0.032))
        NamedCommands.registerCommand('slow front auto fourth shot angle drop', self.angulator.angulator_set_pos_cmd(0.036))
        NamedCommands.registerCommand('prepare slow front auto fifth', self.auto_intake_cmd())
        NamedCommands.registerCommand('prepare slow front auto sixth', self.auto_intake_cmd())
        NamedCommands.registerCommand('auto intake', self.auto_intake_cmd())

        NamedCommands.registerCommand('prepare racer 1', self.auto_prep_fast(65, 0.082))
        NamedCommands.registerCommand('prepare racer 2', self.auto_prep_fast(80, 0.002))
        NamedCommands.registerCommand('prepare racer 3', self.auto_prep_fast(80, 0.009))
        NamedCommands.registerCommand('prepare racer 4', self.auto_shoot_fast(65, 0.09))
        NamedCommands.registerCommand('prepare racer 5', self.auto_shoot_fast(80, 0.041))
        NamedCommands.registerCommand('prepare racer 6', self.auto_shoot_fast(80, 0.036))
        NamedCommands.registerCommand('barf first piece', self.auto_barf_command())
        NamedCommands.registerCommand('intake and shoot', self.intake_and_shoot())
        NamedCommands.registerCommand('intake and shoot off', self.intake_and_shoot_off())
        NamedCommands.registerCommand('enable auto aim', self.angulator.set_auto_aim_enable_cmd(True))

        NamedCommands.registerCommand('prepare first feeder shot', self.prep_first_feeder_shot_auto())
        NamedCommands.registerCommand('prepare first close shot', self.prep_first_close_shot_auto())
        NamedCommands.registerCommand('shooter on', self.shooter.shooter_on_cmd(76))
        NamedCommands.registerCommand('shooter off', self.shooter.shooter_off_cmd())
        NamedCommands.registerCommand('wait for second beam break', 
                                      WaitUntilCommand(lambda: self.beambreak_two.get() == False).withTimeout(1.0))
        NamedCommands.registerCommand('wait for first beam break', 
                                      WaitUntilCommand(lambda: self.beambreak_one.get() == False).withTimeout(2.0))
        NamedCommands.registerCommand('wait for not second beam break', 
                                      WaitUntilCommand(lambda: 
                                                       #(self.beambreak_two.get() == True) and 
                                                       (self.beambreak_one.get() == True)).withTimeout(1.0))
        NamedCommands.registerCommand('wait for shooter on target', self.shooter.wait_for_shooter_on_target_auto())
        NamedCommands.registerCommand('wait for angulator on target', self.angulator.wait_for_angulator_on_target())
        NamedCommands.registerCommand('auto shooter command', 
                                      sequence(
                                          RunCommand(lambda: self.shooter.set_speed_from_range(self.drivetrain.get_range_to_speaker)),
                                          RunCommand(lambda: self.angulator.set_pos_from_range(self.drivetrain.get_range_to_speaker))))
        
        NamedCommands.registerCommand('enable_note_correction', self.drivetrain.set_correct_path_to_note_cmd(True))
        NamedCommands.registerCommand('disable_note_correction', self.drivetrain.set_correct_path_to_note_cmd(False))
        
        NamedCommands.registerCommand('enable_face_speaker', self.drivetrain.set_auto_face_goal_cmd(True))
        NamedCommands.registerCommand('disable_face_speaker', self.drivetrain.set_auto_face_goal_cmd(False))
        NamedCommands.registerCommand('RaisedIntakeOn', self.auto_intake_cmd_raised())
        
        self.chooser = SendableChooser()
        self.chooser.setDefaultOption("Source 4 RB", PathPlannerAuto("Slow Source Auto"))
        self.chooser.addOption("4 Close RB", PathPlannerAuto("Close 4 Piece"))
        self.chooser.addOption("Slow 6 Piece RB that works", PathPlannerAuto("Slow 6 piece"))
        self.chooser.addOption("Racer 5 Piece", PathPlannerAuto("Racer 5 Piece"))

        SmartDashboard.putData("Auto", self.chooser)
        
        self.beambreak_one = DigitalInput(constants.kFeederBeambreakStageOnePort)
        self.beambreak_trigger_one = Trigger(self.beambreak_one.get)
        self.beambreak_trigger_one.onFalse(self.beambreak_one_false_cmd())

        self.beambreak_two = DigitalInput(constants.kFeederBeambreakStageTwoPort)
        self.beambreak_trigger_two = Trigger(self.beambreak_two.get)
        self.beambreak_trigger_two.onFalse(self.beambreak_two_false_cmd())
        
        self.candle = CANdle(1, '2481')
        
        self.align_state = constants.kAlignStateSpeaker       
         
        self.driver_controller = CommandXboxController(
            constants.kDriverControllerPort)
        self.operator_controller = CommandXboxController(            
            constants.kOperatorControllerPort)
        
        self.diag_controller = CommandXboxController(            
            constants.kDiagControllerPort)
    
        self.button_bindings_configure()
        self.drivetrain.setDefaultCommand(self.drivetrain.drive_with_joystick_cmd(self.driver_controller))
        self.angulator.setDefaultCommand(self.angulator.angulator_set_pos_from_range_cmd(self.drivetrain.get_range_to_speaker))
        self.shooter.setDefaultCommand(self.shooter.shooter_default_cmd(self.drivetrain.get_range_to_speaker))
        
        SmartDashboard.putNumber("shooter_cmd_angle", 0)
        SmartDashboard.putNumber("shooter_cmd_speed", 0)
        # DataLogManager.start()

        self.auto_path = PathPlannerAuto("Red 6 piece")
        self.prev_alliance = None
        
        self.__prev_beam_break = True
        self.robot_loop_time_prev = 0
        
        SignalLogger.stop()


    def button_bindings_configure(self):
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        self.operator_controller.a().onTrue(self.prepare_happy_donut_cmd())
        self.operator_controller.x().onTrue(self.prepare_subwoofer_shot_cmd())
        self.operator_controller.y().onTrue(self.prepare_feed_shot_cmd())
        self.operator_controller.rightBumper().whileTrue(self.prepare_speaker_shot_cmd())
        self.operator_controller.rightBumper().onFalse(self.angulator.set_auto_aim_enable_cmd(False)\
            .alongWith(self.shooter.shooter_off_cmd())\
            .alongWith(self.angulator.angulator_set_pos_cmd(constants.kAngulatorDownPosition)))
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
        self.driver_controller.rightTrigger().onTrue(self.intake_feeder_cmd(constants.kTeleopFeederSpeed,
                                                                            constants.kHorizontalIntakeMotorDutyCycle, 
                                                                            constants.kVerticalIntakeMotorDutyCycle))        
        self.driver_controller.leftTrigger().whileTrue(self.drivetrain.drive_speaker_aligned_cmd(self.driver_controller))
        self.driver_controller.leftBumper().onTrue(self.speaker_score_cmd())
        
        #Use for sysID Test
        self.driver_controller.povUp().onTrue(self.angulator.angulator_up_cmd())
        self.driver_controller.povDown().onTrue(self.angulator.angulator_down_cmd())
        
        self.diag_controller.povLeft().whileTrue(self.drivetrain.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kReverse))
        self.diag_controller.povRight().whileTrue(self.drivetrain.sysid_quasistatic_cmd(sysid.SysIdRoutine.Direction.kForward))
        self.diag_controller.povUp().whileTrue(self.drivetrain.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kForward))
        self.diag_controller.povDown().whileTrue(self.drivetrain.sysid_dynamic_cmd(sysid.SysIdRoutine.Direction.kReverse))

        SmartDashboard.putData("Reset Odom", InstantCommand(lambda: self.drivetrain.reset_pose()).ignoringDisable(True))
        SmartDashboard.putData("Zero Steer Encoder", self.drivetrain.zero_steer_encoder_cmd())
        # SmartDashboard.putData("Calibrate Wheel Circumference", self.drivetrain.calibrate_wheel_circumference_cmd())
        # SmartDashboard.putData("Reset Odom To Vision", self.drivetrain.reset_odom_to_vision_cmd().ignoringDisable(True))
        SmartDashboard.putData("+shoot speed", self.shooter.increase_shooter_speed_cmd()) 
        SmartDashboard.putData("-shoot speed", self.shooter.decrease_shooter_speed_cmd())       
        SmartDashboard.putData("Zero Angulator", self.angulator.zero_angulator_encoder_cmd().ignoringDisable(True))
        
        SmartDashboard.putData("Note Correct Enable", self.drivetrain.set_correct_path_to_note_cmd(True))
        SmartDashboard.putData("Note Correct Disable", self.drivetrain.set_correct_path_to_note_cmd(False))
        
    def ignore_beam_break_cmd(self, enabled):
        return InstantCommand(lambda: self.change_beam_break(enabled))
    
    def change_beam_break(self, enabled):
        self.IgnoreBeamBreaks = enabled
        
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
    
    def prepare_feed_shot_cmd(self):
        return(# angulator to feed position 
               self.angulator.angulator_set_pos_cmd(constants.kAngulatorFeedAngleDeg)
               # shooter on to feed persentage
               .alongWith(self.shooter.shooter_on_cmd(constants.kShooterSpeedFeedRPS)))

    def set_align_state(self, state):
        self.align_state = state


    def set_align_state_cmd(self, state):
        return runOnce(lambda: self.set_align_state(state))


    def set_angle_speed_from_range(self):
        self.angulator.set_auto_aim_enable(True)
        self.shooter.set_shooter_auto_enable(True)
        # self.angulator.set_pos_from_range(self.drivetrain.get_range_to_speaker)
        # self.shooter.set_speed_from_range(self.drivetrain.get_range_to_speaker)
    

    def prepare_speaker_shot_cmd(self):
        return (
            FunctionalCommand(lambda: self.set_align_state(constants.kAlignStateSpeaker),
                              lambda: self.set_angle_speed_from_range(),
                              lambda interrupted: None,
                              lambda: None, #self.driver_controller.leftBumper().getAsBoolean()
                                # self.angulator  We may need to put this back.
                            )
                )
    
    def prepare_auto_speaker_shot_cmd(self):
        return (
            FunctionalCommand(lambda: self.set_align_state(constants.kAlignStateSpeaker),
                              lambda: self.set_angle_speed_from_range(),
                              lambda interrupted: None,
                              lambda: self.shooter.shooterMotor.get_closed_loop_error().value < constants.kShooterOnTarget)
                                # self.angulator)
                )
    
    def prepare_auto_shooter_and_angulator_cmd(self, shooter_rps, shooter_angle):
        return parallel(
           self.shooter.shooter_on_cmd(shooter_rps),
           self.angulator.angulator_set_pos_cmd(shooter_angle)
        )



    def speaker_score_cmd(self):
            return (sequence(
                # self.angulator.wait_for_angulator_on_target(),
                self.shooter.wait_for_shooter_on_target(),
                self.ignore_beam_break_cmd(True),
                self.feeder.feeder_on_cmd(.9),
                WaitCommand(0.2),
                self.ignore_beam_break_cmd(False),
                self.feeder.feeder_off_cmd(),
                self.shooter.shooter_off_cmd(),
                self.angulator.angulator_set_pos_cmd(0)
        ))
               
    def amp_handoff_cmd(self):
        return (sequence(
                self.set_align_state_cmd(constants.kAlignStateAmp),
                self.ignore_beam_break_cmd(True),
                self.intake.set_intake_cmd(0, 0),
                self.feeder.feeder_on_cmd(0),
                self.shooter.shooter_on_cmd(25),
                self.angulator.angulator_set_pos_cmd_amp_only(0.0),
                self.arm.arm_pickup_pos_cmd(constants.kArmPickupPosition),
                self.angulator.angulator_amp_handoff_cmd().withTimeout(1.0),
                self.feeder.feeder_on_cmd(.95),
                self.arm.gripper_close_cmd(),
                WaitCommand(.1),
                self.arm.arm_score_pos_cmd().withTimeout(1.0),
                WaitCommand(0.5),
                self.ignore_beam_break_cmd(False),
                self.shooter.shooter_off_cmd(),
                self.feeder.feeder_off_cmd(),
                self.angulator.angulator_set_pos_cmd_amp_only(0)))


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
            self.angulator.angulator_set_pos_cmd(0.012)))
            #0.014 works with fresh battery
    

    def prep_first_feeder_shot_auto(self):
        return(sequence(
            self.shooter.shooter_on_cmd(65),
            self.angulator.angulator_set_pos_cmd(0.011)))
    
    def prep_first_close_shot_auto(self):
        return(sequence(
            self.shooter.shooter_on_cmd(65),
            self.angulator.angulator_set_pos_cmd(0.08)))

    def getAutonomousCommand(self):
        return self.chooser.getSelected()
    #self.auto_path

    
    def intake_sequence_cmd(self, feeder_cmd, horizontal, vertical, auto=False):
        return sequence(
                        self.ignore_beam_break_cmd(False),
                        self.angulator.angulator_set_pos_cmd(0.0),
                        self.intake.set_intake_cmd(horizontal, vertical),
                        self.feeder.feeder_on_cmd(feeder_cmd)
                        )


    def vomit_cmd(self):
        return (sequence(
                self.shooter.shooter_on_cmd(-50),
                self.intake_sequence_cmd(-0.5, -0.5, -0.5)
                ))


    def intake_feeder_cmd(self, feeder_cmd, intake_cmd_horizontal, intake_cmd_vertical, auto=False):
        return(sequence(
            self.ignore_beam_break_cmd(False),
            self.set_align_state_cmd(constants.kAlignStateNote),
            self.intake_sequence_cmd(feeder_cmd, intake_cmd_horizontal, intake_cmd_vertical, auto))
            )
    
        
    def beambreak_during_intake_cmd(self):
        return (
            #  InstantCommand(lambda: SmartDashboard.putNumber("beambreak one", False)).alongWith(
             InstantCommand(lambda: self.intake.horizontalMotor.set_control(VoltageOut(0))).alongWith(
             self.feeder.feeder_on_cmd(0.07)) #0.1
        )

    def is_beam_break_ignored(self) -> bool:
        return self.IgnoreBeamBreaks == True

    def beambreak_one_false_cmd(self):
        return ConditionalCommand(InstantCommand(lambda: None),
                                  self.beambreak_during_intake_cmd(), 
                                  self.is_beam_break_ignored)
    
    def beambreak_two_false_cmd(self):
        return ConditionalCommand(InstantCommand(lambda: None),
                                  InstantCommand(lambda: self.feeder.feederMotor.set_control(VoltageOut(0))).alongWith(
                                  InstantCommand(lambda: self.intake.horizontalMotor.set_control(VoltageOut(0)))).alongWith(
                                  InstantCommand(lambda: self.intake.verticalMotor.set_control(VoltageOut(0)))),
                                  self.is_beam_break_ignored)

    ##AUTO COMMANDS
    
    def auto_intake_cmd(self):
        return sequence(
                        # self.drivetrain.set_correct_path_to_note_cmd(True),
                        self.ignore_beam_break_cmd(False),
                        self.angulator.angulator_set_pos_cmd(0),
                        self.intake.set_intake_cmd(constants.kHorizontalIntakeMotorDutyCycle, constants.kVerticalIntakeMotorDutyCycle),
                        self.feeder.feeder_on_cmd(constants.kTeleopFeederSpeed),
                        #beambreak trigger will turn off intake
                        WaitUntilCommand(lambda: self.beambreak_one.get() == False).withTimeout(1.0))  
        
    def intake_and_shoot(self):
        return sequence(
                        # self.drivetrain.set_correct_path_to_note_cmd(True),
                        self.ignore_beam_break_cmd(True),
                        self.angulator.angulator_set_pos_cmd(0.04),
                        self.intake.set_intake_cmd(constants.kHorizontalIntakeMotorDutyCycleRaisedAuto, constants.kVerticalIntakeMotorDutyCycleRaisedAuto),
                        self.feeder.feeder_on_cmd(constants.kTeleopFeederSpeedRaisedAuto),
                        self.shooter.shooter_on_cmd(65))
        
    def intake_and_shoot_off(self):
        return sequence(
                        # self.drivetrain.set_correct_path_to_note_cmd(True),
                        self.ignore_beam_break_cmd(False),
                        self.angulator.angulator_set_pos_cmd(0.01),
                        self.intake.set_intake_cmd(0, 0),
                        self.feeder.feeder_on_cmd(0),
                        self.shooter.shooter_on_cmd(0))
        
    def auto_intake_cmd_raised(self):
        return sequence(
                        # self.drivetrain.set_correct_path_to_note_cmd(True),
                        self.angulator.angulator_set_pos_cmd(0.04),
                        self.intake.set_intake_cmd(constants.kHorizontalIntakeMotorDutyCycleRaisedAuto, constants.kVerticalIntakeMotorDutyCycleRaisedAuto),
                        self.feeder.feeder_on_cmd(constants.kTeleopFeederSpeedRaisedAuto),
                        #beambreak trigger will turn off intake
                        WaitUntilCommand(lambda: self.beambreak_one.get() == False).withTimeout(1.0))                     
                        
    def auto_prep_shoot_cmd(self, shooter_speed, angulator_position):
        return sequence(sequence(
            self.shooter.shooter_on_cmd(shooter_speed),
            self.angulator.angulator_set_pos_cmd(angulator_position))                    
        )
        
    def auto_shoot_cmd(self):    
        return sequence(self.shooter.wait_for_shooter_on_target_auto().alongWith(self.angulator.wait_for_angulator_on_target()),
                        self.feeder.feeder_on_cmd(.9),
                        WaitUntilCommand(lambda: self.beambreak_one.get() == True).withTimeout(1.0),
                        self.feeder.feeder_off_cmd(),
                        )   
        
    def no_wait_auto_shoot_cmd(self):    
        return sequence(self.feeder.feeder_on_cmd(.9),
                        WaitUntilCommand(lambda: self.beambreak_one.get() == True).withTimeout(1.0),
                        self.feeder.feeder_off_cmd(),
                        )  
                     
    def auto_shoot_fast(self, shooter_speed, angulator_position):
        return sequence(self.auto_intake_cmd(),
                        self.auto_prep_shoot_cmd(shooter_speed, angulator_position),
                        self.auto_shoot_cmd())
        
        
    def no_wait_auto_shoot_fast(self, shooter_speed, angulator_position):
        return sequence(self.auto_intake_cmd(),
                        self.auto_prep_shoot_cmd(shooter_speed, angulator_position),
                        self.no_wait_auto_shoot_cmd())
                            
                        
    def auto_prep_fast(self, shooter_speed, angulator_position):
        return sequence(self.auto_intake_cmd(),
                        self.auto_prep_shoot_cmd(shooter_speed, angulator_position))
        
    def auto_prep_fast_raised(self, shooter_speed, angulator_position):
        return sequence(self.auto_intake_cmd_raised(),
                        self.auto_prep_shoot_cmd(shooter_speed, angulator_position))
        
    def auto_barf_command(self):
        return sequence(self.shooter.shooter_on_cmd(5),
                        WaitCommand(0.2),
                        self.shooter.shooter_off_cmd())         
                      
    def periodic(self):
        end_time = wpilib.Timer.getFPGATimestamp()
        dt = end_time - self.robot_loop_time_prev
        if dt < 0.2:
            SmartDashboard.putNumber("robot_loop", dt)
        self.robot_loop_time_prev = end_time  
        
        # SmartDashboard.putData("Scheduler", CommandScheduler.getInstance())
        # SmartDashboard.putNumber("speak range", self.drivetrain.get_range_to_speaker())
        # SmartDashboard.putNumber("shoot speed", self.shooter.shooterMotor.get_velocity().value)
        # SmartDashboard.putNumber("shoot speed ref", self.shooter.shooterMotor.get_closed_loop_reference().value)
        # SmartDashboard.putNumber("shoot speed motor voltage", self.shooter.shooterMotor.get_motor_voltage().value)
        # SmartDashboard.putNumber("shoot supply voltage", self.shooter.shooterMotor.get_supply_voltage().value)

        # SmartDashboard.putNumber("feeder duty cycle", self.feeder.feederMotor.get_duty_cycle().value)
        # SmartDashboard.putNumber("shooter current", self.shooter.shooterMotor.get_supply_current().value)

        # SmartDashboard.putNumber("beambreak one", self.beambreak_one.get())
        # SmartDashboard.putNumber("beambreak two", self.beambreak_two.get())

        # Make sure we are connected to FMS / DS before building auto so we get the alliance color correct.
        # if DriverStation.getAlliance() != self.prev_alliance:
        #     # if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        #     self.auto_path = PathPlannerAuto("Slow Source Auto")
        #     # else:
        #         # self.auto_path = PathPlannerAuto("6 piece")

        #     self.prev_alliance = DriverStation.getAlliance()
        beam_break = self.beambreak_one.get()
        if beam_break != self.__prev_beam_break:
            if not beam_break:
                self.candle.setLEDs(255, 255, 255) # LED's On Green 0 - 8
            else:
                self.candle.setLEDs(0, 0, 0) # LED's Off 0 - 8  
        self.__prev_beam_break = beam_break   
         
        


    
