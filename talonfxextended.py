#!/usr/bin/env python3
"""
    This is a demo program for TalonFX usage in Phoenix 6
"""
import time
import wpilib
from wpilib import Timer, XboxController
from phoenix6 import hardware, unmanaged, controls
from phoenix6 import hardware, controls, configs, unmanaged


from phoenix6.hardware.talon_fx import TalonFX;
from phoenix6.spns.spn_value import SpnValue
from phoenix6.units import *
from phoenix6.status_signal import *

class TalonFXExtended(TalonFX):
    """
    Extended to implement missing getters.
    -OZ
    """

    #------------------------------------------#
    def get_closedloop_reference_position(self) -> StatusSignal[float]:
        """
        Only call when using position closed loop.
        """
        return self._common_lookup(SpnValue.PRO_PIDREF_PIDERR_PIDREF_POSITION.value, 0, None, "reference (position)", True, float)
    def get_closedloop_reference_velocity(self) -> StatusSignal[float]:
        """
        Only call when using velocity closed loop.
        """
        return self._common_lookup(SpnValue.PRO_PIDREF_PIDERR_PIDREF_VELOCITY.value, 0, None, "reference (velocity)", True, float)
    #------------------------------------------#
    def get_closedloop_reference_slope_position(self) -> StatusSignal[float]:
        """
        Only call when using position closed loop.
        """
        return self._common_lookup(SpnValue.PRO_PIDREF_SLOPE_ECUTIME_REFERENCE_SLOPE_POSITION.value, 0, None, "reference slope (position)", True, float)
    def get_closedloop_reference_slope_velocity(self) -> StatusSignal[float]:
        """
        Only call when using velocity closed loop.
        """
        return self._common_lookup(SpnValue.PRO_PIDREF_SLOPE_ECUTIME_REFERENCE_SLOPE_VELOCITY.value, 0, None, "reference slope (velocity)", True, float)
    
    #------------------------------------------#
    def get_closedloop_error_position(self) -> StatusSignal[float]:
        """
        Only call when using position closed loop.
        """
        return self._common_lookup(SpnValue.PRO_PIDREF_PIDERR_PIDERR_POSITION.value, 0, None, "error (position)", True, float)
    def get_closedloop_error_velocity(self) -> StatusSignal[float]:
        """
        Only call when using velocity closed loop.
        """
        return self._common_lookup(SpnValue.PRO_PIDREF_PIDERR_PIDERR_VELOCITY.value, 0, None, "error (velocity)", True, float)
    #------------------------------------------#

    def get_closedloop_output_dutycycle(self) -> StatusSignal[float]:
        """
        Only call when using dutycyle closed loop.
        """
        return self._common_lookup(SpnValue.PRO_PIDOUTPUT_OUTPUT_DC.value, 0, None, "output (duty cycle)", True, float)
    def get_closedloop_output_voltage(self) -> StatusSignal[float]:
        """
        Only call when using voltage closed loop.
        """
        return self._common_lookup(SpnValue.PRO_PIDOUTPUT_OUTPUT_V.value, 0, None, "output (voltage)", True, float)
    def get_closedloop_output_torquecurrent(self) -> StatusSignal[float]:
        """
        Only call when using torquefoc closed loop.
        """
        return self._common_lookup(SpnValue.PRO_PIDOUTPUT_OUTPUT_A.value, 0, None, "output (torque current)", True, float)
    #------------------------------------------#



class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use TalonFX
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the motor controllers used
        self.talonfx = TalonFXExtended(1, "canivore")
        self.control = controls.DutyCycleOut(0)

        self.timer = Timer()
        self.timer.start()

        self.joystick = XboxController(0)

        self.talonfx.sim_state.set_supply_voltage(13)

        self.last_time = time.time()

        self.position_request = controls.MotionMagicVoltage(0)

        cfg = configs.TalonFXConfiguration()
        # Set PID gains
        cfg.slot0.k_p = 1
        cfg.slot0.k_d = 0
        cfg.motion_magic.motion_magic_jerk = 50
        cfg.motion_magic.motion_magic_acceleration = 50
        cfg.motion_magic.motion_magic_cruise_velocity = 50

        # Apply PID gains to motor
        self.talonfx.configurator.apply(cfg)


    def teleopInit(self):
        """Set the position of the talonfx to 0 so we know we're centered"""
        self.talonfx.set_position(0)

    def teleopPeriodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        # Target a position of +- 10 rotations depending on joystick position
        self.talonfx.set_control(self.position_request.with_position(self.joystick.getLeftY() * -10))

        if self.timer.hasElapsed(0.250):
            self.timer.reset()

            pos = self.talonfx.get_position()
            vel = self.talonfx.get_velocity()
            ref = self.talonfx.get_closedloop_reference_position()
            refslope = self.talonfx.get_closedloop_reference_slope_position()
            err = self.talonfx.get_closedloop_error_position()
            output = self.talonfx.get_closedloop_output_voltage();
            
            print(f"{str(pos.name)}: {str(pos)}")
            print(f"{str(vel.name)}: {str(vel)}")
            print(f"{str(ref.name)}: {str(ref)}")
            print(f"{str(refslope.name)}: {str(refslope)}")
            print(f"{str(err.name)}: {str(err)}")
            print(f"{str(output.name)}: {str(output)}")
            print("")

    def _simulationPeriodic(self):
        """"""
        # If the driver station is enabled, then feed enable for phoenix devices
        if wpilib.DriverStation.isEnabled():
            unmanaged.feed_enable(100)

        # threw together a quick simluation -OZ
        now = time.time()
        dur_sec = now - self.last_time

        self.last_time = now

        sim_vel_rps = self.talonfx.get_motor_voltage().value_as_double / 12.0 * 100.0
        
        self.talonfx.sim_state.set_rotor_velocity(sim_vel_rps)
        self.talonfx.sim_state.add_rotor_position(sim_vel_rps * dur_sec)


if __name__ == "__main__":
    wpilib.run(MyRobot)