import wpilib
import wpilib.simulation

import wpimath.system.plant
from pyfrc.physics.core import PhysicsInterface

import math
import typing
import phoenix6
from phoenix6.sim import *
from phoenix6.hardware.core.core_talon_fx import CoreTalonFX
from phoenix6.hardware.core import * 

from phoenix6 import unmanaged

if typing.TYPE_CHECKING:
    from robot import MyRobot

#class Falcon:
   # phoenix6.sim.talon_fx_sim_state.TalonFXSimState (CoreTalonFX): phoenix6.hardware.core.core_talon_fx.CoreTalonFX
    #phoenix6.sim.chassis_reference.ChassisReference = ChassisReference.CounterClockwise_Positive

class MechanismSpinner(object):

    def __init__(self, name, parent, length, color):
        self.__pos = 0
        self.__state = False

        self.spokes = []
        for i in range(6):
            self.spokes.append(
                parent.appendLigament(name + " Spoke %d" % i, length, 0, 6, wpilib.Color8Bit(color))
            )

    def update(self, state):
        self.__state = state

        if self.__state:
            self.__pos += 10
            if self.__pos == 360:
                self.__pos = 0

        for i, spoke in enumerate(self.spokes):
            spoke.setAngle(self.__pos + i * 60)


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.robot = robot

        # # Create a Mechanism2d display of intake
        self.mech2d = wpilib.Mechanism2d(80, 80)

        self.platform = self.mech2d.getRoot("Platform Base", 20, 30)

        # Chasis
        self.chassis = self.mech2d.getRoot("ChassisBase", 5, 30)
        self.chassis_horiz = self.chassis.appendLigament("Chassis Horizontal", 60, 0, 6,  wpilib.Color8Bit(wpilib.Color.kGray))

        #Feeder
        self.feeder_base = self.mech2d.getRoot("FeederBase", 15, 30)
        self.feeder_adjust = self.feeder_base.appendLigament("Feeder Adjust", 20, 30, 6, wpilib.Color8Bit(wpilib.Color.kNavy))
        
        



        # Shooter Pivot
        self.shooter_base = self.mech2d.getRoot("ShooterBase", 15, 30)
        self.shooter_adjust = self.shooter_base.appendLigament("Shooter Adjust", 45, 30, 6, wpilib.Color8Bit(wpilib.Color.kNavy))

        self.intake_base = self.mech2d.getRoot("IntakeBase", 5, 25)
        self.intake_roller = MechanismSpinner("Intake", self.intake_base, 5, wpilib.Color.kWhite)
        self.feeder_roller = MechanismSpinner("Feeder", self.feeder_adjust, 5, wpilib.Color.kOrange)
        self.shooter_roller = MechanismSpinner("Shooter", self.shooter_adjust, 5, wpilib.Color.kGreen)
       
        # self.gripperBase = self.mech2d.getRoot("GripperBase", 20, 50)
        # self.gripperFixed = self.platform.appendLigament(
        #     "Gripper Fixed", 25, 90, 6, wpilib.Color8Bit(wpilib.Color.kBlue)
        # )
        # self.gripperFinger = self.gripperFixed.appendLigament(
        #     "Gripper Finger", 20, 45, 6, wpilib.Color8Bit(wpilib.Color.kYellow)
        # )
        # self.gripperFinger2 = self.gripperFinger.appendLigament(
        #     "Gripper Finger2", 5, -90, 6, wpilib.Color8Bit(wpilib.Color.kYellow)
        # )

        # self.flipperFixed = self.platform_vertical.appendLigament(
        #     "Flipper Fixed", 7, -180, 6, wpilib.Color8Bit(wpilib.Color.kWhite)
        # )

        # self.flipperFixed2 = self.flipperFixed.appendLigament(
        #     "Flipper Fixed2", 3, -90, 6, wpilib.Color8Bit(wpilib.Color.kBlack)
        # )

        # self.flipperJoint = self.flipperFixed2.appendLigament(
        #     "Flipper Joint", 10, 80, 6, wpilib.Color8Bit(wpilib.Color.kRed)
        # )

        # self.flipperFixed3 = self.flipperJoint.appendLigament(
        #     "Flipper Joint", 15, 90, 6, wpilib.Color8Bit(wpilib.Color.kRed)
        # )

        # # self.flipperFinger = self.gripperBase.appendLigament(
        # #     "Flipper Finger", 10, -80, 6,  wpilib.Color8Bit(wpilib.Color.kPurple)
        # # )

        wpilib.SmartDashboard.putData("Robot Sim", self.mech2d)

        # self.gripper_pos = 20
        # self.gripper_fwd = True

        # # # self.armBase = self.mech2d.getRoot("ArmBase", 30, 30)
        # # self.armTower = self.armBase.appendLigament(
        # #     "Arm Tower", 30, -90, 6, wpilib.Color8Bit(wpilib.Color.kBlue)
        # # )
        # # self.arm = self.armBase.appendLigament(
        # "Arm", 30, self.armSim.getAngle(), 6, wpilib.Color8Bit(wpilib.Color.kYellow)
        # # )

        # # Put Mechanism to SmartDashboard
        # # wpilib.SmartDashboard.putData("Arm Sim", self.mech2d)

        # # self.flipperSim = self.robot.container.flipper.flipperMotor.get_sim

    def update_angulator(self):
        
        if self.robot.container.angulator.angulatorMotor.sim_state.motor_voltage > 0.0:
            if self.shooter_adjust.getAngle() < 70:
                self.shooter_adjust.setAngle(self.shooter_adjust.getAngle()+1)
                self.feeder_adjust.setAngle(self.shooter_adjust.getAngle())
                self.robot.container.angulator.angulatorMotor.sim_state.set_raw_rotor_position(100)
                self.robot.container.angulator.angulatorMotor.sim_state.set_rotor_velocity(10000)
        elif self.robot.container.angulator.angulatorMotor.sim_state.motor_voltage < 0.0:
            if self.shooter_adjust.getAngle() > 25:
                self.shooter_adjust.setAngle(self.shooter_adjust.getAngle()-1) 
                self.feeder_adjust.setAngle(self.shooter_adjust.getAngle())
                self.robot.container.angulator.angulatorMotor.sim_state.set_raw_rotor_position(100)
                self.robot.container.angulator.angulatorMotor.sim_state.set_rotor_velocity(10000)




    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        if wpilib.DriverStation.isEnabled():
            self.robot.container.intake.horizontalMotor.sim_state.set_supply_voltage(12.0)
            self.robot.container.shooter.shooterMotor.sim_state.set_supply_voltage(12.0)
            self.robot.container.feeder.feederMotor.sim_state.set_supply_voltage(12.0)
            self.robot.container.angulator.angulatorMotor.sim_state.set_supply_voltage(12.0)
        else:
            self.robot.container.intake.horizontalMotor.sim_state.set_supply_voltage(0.0)
            self.robot.container.shooter.shooterMotor.sim_state.set_supply_voltage(0.0)
            self.robot.container.feeder.feederMotor.sim_state.set_supply_voltage(0.0)
            self.robot.container.angulator.angulatorMotor.sim_state.set_supply_voltage(0.0)

        self.intake_roller.update(self.robot.container.intake.horizontalMotor.sim_state.motor_voltage > 0)
        self.shooter_roller.update(self.robot.container.shooter.shooterMotor.sim_state.motor_voltage > 0)
        self.feeder_roller.update(self.robot.container.feeder.feederMotor.sim_state.motor_voltage > 0)

        self.update_angulator()

        # Feed the enable signal to all motors.  No motors will turn in simulation without this.
        if wpilib.DriverStation.isEnabled():
            unmanaged.feed_enable(100)