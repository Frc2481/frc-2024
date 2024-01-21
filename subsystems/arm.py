import math
import wpilib
from wpilib import DoubleSolenoid

import constants

import commands2
import commands2.cmd

class ArmSubsystem(object):

    def __init__(self):
        super().__init__()

        self.armSolenoid = DoubleSolenoid(
            constants.ARM_SOLENOID_MODULE,
            moduleType = wpilib.PneumaticsModuleType.REVPH,
            forwardChannel = constants.ARM_DOUBLE_SOLENOID_FORWARD_PORT,
            reverseChannel = constants.ARM_DOUBLE_SOLENOID_REVERSE_PORT
        )

    def arm_extend_cmd(self):
        return commands2.cmd.runOnce(
            lambda: self.armSolenoid.set(DoubleSolenoid.Value.kForward)
        )
    
    def arm_retract_cmd(self):
        return commands2.cmd.runOnce(
            lambda: self.armSolenoid.set(DoubleSolenoid.Value.kReverse)
        )
