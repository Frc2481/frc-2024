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
            constants.kArmSolenoidModule,
            moduleType = wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel = constants.kArmDoubleSolenoidForwardPort,
            reverseChannel = constants.kArmDoubleSolenoidReversePort
        )

    def arm_extend_cmd(self):
        return commands2.cmd.runOnce(
            lambda: self.armSolenoid.set(DoubleSolenoid.Value.kForward)
        )
    
    def arm_retract_cmd(self):
        return commands2.cmd.runOnce(
            lambda: self.armSolenoid.set(DoubleSolenoid.Value.kReverse)
        )
