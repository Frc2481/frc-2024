import math

import wpilib
from commands2 import *
from commands2.cmd import * 

import constants

import phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityTorqueCurrentFOC, VoltageOut, VelocityVoltage, VelocityDutyCycle, MotionMagicVelocityVoltage
from phoenix6.signals.spn_enums import *
from phoenix6.status_signal import BaseStatusSignal

from wpilib import SmartDashboard

class ShooterSubsystem(Subsystem):

    def __init__(self):
        super().__init__()

        self.setpoint = 0
        
        self.auto_set_shooter = False

        self.shooterMotor = TalonFX(constants.kShooterMotorCANID, "2481")

        self.shooterMotorConfig = TalonFXConfiguration()
        self.shooterMotorConfig.current_limits.stator_current_limit = 80
        self.shooterMotorConfig.current_limits.stator_current_limit_enable = True
        self.shooterMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.shooterMotorConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.shooterMotorConfig.slot0.k_p = constants.kShooterP
        self.shooterMotorConfig.slot0.k_i = constants.kShooterI
        self.shooterMotorConfig.slot0.k_d = constants.kShooterD
        self.shooterMotorConfig.slot0.k_v = constants.kShooterV
        self.shooterMotorConfig.slot0.k_a = constants.kShooterA
        self.shooterMotorConfig.slot0.k_s = constants.kShooterS
        self.shooterMotorConfig.motion_magic.motion_magic_cruise_velocity = constants.kShooterCruiseVelocity
        self.shooterMotorConfig.motion_magic.motion_magic_acceleration = constants.kShooterAcceleration
        self.shooterMotorConfig.motion_magic.motion_magic_jerk = constants.kShooterJerk
        self.shooterMotorConfig.torque_current.peak_forward_torque_current

        self.shooterMotor.configurator.apply(self.shooterMotorConfig)

        self.velocity_control_request = VelocityDutyCycle(0)
        self.off_control_request = VoltageOut(0)
        
        self.closed_loop_error_signal = self.shooterMotor.get_closed_loop_reference()
        self.velocity_signal = self.shooterMotor.get_velocity()
        
        self.allSignals = [self.closed_loop_error_signal,
                           self.velocity_signal]

    def shooter_set_setpoint(self, sp):
        self.setpoint = sp


    def shooter_on_cmd(self, shooter_speed_rps = constants.kShooterSpeedHappyDonutRPS):
        return runOnce(
            lambda: self.shooterMotor.set_control(self.velocity_control_request.with_velocity(shooter_speed_rps))
        ).andThen(InstantCommand(lambda: self.shooter_set_setpoint(shooter_speed_rps)))


    def shooter_off_cmd(self):
        #return InstantCommand(lambda: None)
        return runOnce(lambda: self.set_shooter_auto_enable(False))\
            .alongWith(runOnce(lambda: self.shooterMotor.set_control(self.off_control_request)))\
            .andThen(InstantCommand(self.shooter_set_setpoint(0)))


    def shooter_inc_speed_target(self, delta):
        self.setpoint = self.setpoint + delta
        if self.setpoint > 83:
            self.setpoint = 83

        elif self.setpoint < 0:
            self.setpoint = 0
        self.shooterMotor.set_control(self.velocity_control_request.with_velocity(velocity=self.setpoint))


    def increase_shooter_speed_cmd(self):
        return FunctionalCommand(
            lambda: self.shooter_inc_speed_target(1),
            lambda: None,
            lambda interrupted: None,
            lambda: math.fabs(self.get_error()) < 0.5,
            self
        )


    def is_auto_enabled(self):
        return self.auto_set_shooter
    
    def shooter_default_thing(self, range_cb):
            if self.auto_set_shooter:
                self.set_speed_from_range(range_cb)
           
        
    def shooter_default_cmd(self, range_cb):
        return runEnd(lambda: self.shooter_default_thing(range_cb),
                      lambda: None,
                      self)
        
    def set_shooter_auto_enable(self, enable):
        self.auto_set_shooter = enable
    
    def decrease_shooter_speed_cmd(self):
        return FunctionalCommand(
            lambda: self.shooter_inc_speed_target(-1),
            lambda: None,
            lambda interrupted: None,
            lambda: math.fabs(self.get_error()) < 0.5,
            self
        )


    def get_error(self):
        return self.setpoint - self.velocity_signal.value


    def set_speed_from_range(self, range_cb):
        rps = constants.kShooterSpeedSubwooferRPS
        rps = rps + 4.1 * (range_cb()-1.3)
        if rps > constants.kShooterSpeedMaxRPS:
            rps = constants.kShooterSpeedMaxRPS
        if range_cb() < 1: 
            rps = constants.kShooterSpeedSubwooferRPS
        self.setpoint = rps
        self.shooterMotor.set_control(self.velocity_control_request.with_velocity(rps))



    def shooter_set_speed_from_range_cmd(self, range_cb):
        return runOnce(lambda: self.set_speed_from_range(range_cb))


    def wait_for_shooter_on_target(self):    
        return(sequence(WaitUntilCommand(lambda: self.closed_loop_error_signal.value - constants.kShooterOnTarget < self.velocity_signal.value),
                        PrintCommand('Shooter On Target')))
        
    def wait_for_shooter_on_target_auto(self):    
        return(sequence(WaitUntilCommand(lambda: self.closed_loop_error_signal.value - constants.kShooterOnTarget < self.velocity_signal.value).withTimeout(0.5),
                        PrintCommand('Shooter On Target')))
    
    def periodic(self) -> None:
        BaseStatusSignal.refresh_all(self.allSignals)
        
        SmartDashboard.putNumber("Shooter Speed", self.velocity_signal.value)
        