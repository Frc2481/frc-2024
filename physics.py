"""Robot Physics Module"""
import typing
import wpilib
import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface


if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.robot = robot

        self.mech2d = wpilib.Mechanism2d(90, 90)
        self.physics_controller = physics_controller

        self.platform = self.mech2d.getRoot("Platform Base", 20, 30)
        wpilib.SmartDashboard.putData("Robot Sim", self.mech2d)


        # # self.gripperBase = self.mech2d.getRoot("GripperBase", 20, 50)
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

        # wpilib.SmartDashboard.putData("Gripper Sim", self.mech2d)

        # self.gripper_pos = 20
        # self.gripper_fwd = True

        # # # self.armBase = self.mech2d.getRoot("ArmBase", 30, 30)
        # # self.armTower = self.armBase.appendLigament(
        # #     "Arm Tower", 30, -90, 6, wpilib.Color8Bit(wpilib.Color.kBlue)
        # # )
        # # self.arm = self.armBase.appendLigament(
        # #     "Arm", 30, self.armSim.getAngle(), 6, wpilib.Color8Bit(wpilib.Color.kYellow)
        # # )

        # # Put Mechanism to SmartDashboard
        # # wpilib.SmartDashboard.putData("Arm Sim", self.mech2d)

        # # self.flipperSim = self.robot.container.flipper.flipperMotor.get_sim

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
