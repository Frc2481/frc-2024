import wpilib
import wpilib.simulation
import constants

import wpimath.system.plant
from pyfrc.physics.core import PhysicsInterface

import math
import typing
import phoenix6
from wpilib.simulation import DCMotorSim
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor

from phoenix6 import unmanaged
from phoenix6.hardware import TalonFX

if typing.TYPE_CHECKING:
    from robot import MyRobot

    
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
        
        self.angulator_motor_sim = SingleJointedArmSim(DCMotor.krakenX60(1), 
                                                        constants.kAngulatorGearReduction,
                                                        SingleJointedArmSim.estimateMOI(constants.kArmMass, constants.kArmLength),
                                                        constants.kArmLength,
                                                        math.radians(constants.kAngulatorDownPosition),
                                                        math.radians(constants.kAngulatorUpPosition),
                                                        False,
                                                        0)

        # Shooter Pivot
        self.shooter_base = self.mech2d.getRoot("ShooterBase", 15, 30)
        self.shooter_adjust = self.shooter_base.appendLigament("Shooter Adjust", 45, 30, 6, wpilib.Color8Bit(wpilib.Color.kNavy))

        self.intake_base = self.mech2d.getRoot("IntakeBase", 5, 25)
        self.intake_roller = MechanismSpinner("Intake", self.intake_base, 5, wpilib.Color.kWhite)
        self.feeder_roller = MechanismSpinner("Feeder", self.feeder_adjust, 5, wpilib.Color.kOrange)
        self.shooter_roller = MechanismSpinner("Shooter", self.shooter_adjust, 5, wpilib.Color.kGreen)
       

        wpilib.SmartDashboard.putData("Robot Sim", self.mech2d)


    def update_talonFX(self, motor:TalonFX, motor_sim:SingleJointedArmSim, tm_diff):

        motor_sim.setInputVoltage(motor.sim_state.motor_voltage)
        
        motor_sim.update(tm_diff)

        motor.sim_state.set_raw_rotor_position(motor_sim.getAngleDegrees() / 360 * constants.kAngulatorGearReduction)
        motor.sim_state.set_rotor_velocity(motor_sim.getVelocityDps() / 360 * constants.kAngulatorGearReduction)
        motor.sim_state.set_supply_voltage(12.0) # - motor.sim_state.supply_current * constants.kMotorResistance)
                     

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

        self.update_talonFX(self.robot.container.angulator.angulatorMotor, self.angulator_motor_sim, tm_diff)
        self.shooter_adjust.setAngle(self.angulator_motor_sim.getAngleDegrees())
        self.feeder_adjust.setAngle(self.angulator_motor_sim.getAngleDegrees())

        # Feed the enable signal to all motors.  No motors will turn in simulation without this.
        if wpilib.DriverStation.isEnabled():
            unmanaged.feed_enable(100)