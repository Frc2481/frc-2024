import math
import wpilib
from wpilib import DoubleSolenoid

import constants

import commands2
import commands2.cmd
from commands2 import InstantCommand


class ArmSubsystem(object):

    def __init__(self):
        super().__init__()

        self.armSolenoid = DoubleSolenoid(
            constants.kArmSolenoidModule,
            moduleType = wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel = constants.kArmDoubleSolenoidForwardPort,
            reverseChannel = constants.kArmDoubleSolenoidReversePort
        )
        
        
        self.armSolenoid2 = DoubleSolenoid(
            constants.kArmSolenoidModule,
            moduleType = wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel = constants.kArmDoubleSolenoid2ForwardPort,
            reverseChannel = constants.kArmDoubleSolenoid2ReversePort
       )
        
        self.gripperSolenoid = DoubleSolenoid(
            constants.kGripperSolenoidModule,
            moduleType = wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel = constants.kGripperDoubleSolenoidForwardPort,
            reverseChannel = constants.kGripperDoubleSolenoidReversePort
        )
        
        

    def arm_score_position_cmd(self):
        return commands2.cmd.runOnce(
            lambda: self.armSolenoid.set(DoubleSolenoid.Value.kForward)) \
            .andThen(InstantCommand(lambda: self.armSolenoid2.set(DoubleSolenoid.Value.kForward))
                    
        )
        
    def arm_pickup_position_cmd(self):
        return commands2.cmd.runOnce(
            lambda: self.armSolenoid.set(DoubleSolenoid.Value.kForward)) \
            .andThen(InstantCommand(lambda: self.armSolenoid2.set(DoubleSolenoid.Value.kReverse))
        )   
    
    def arm_stow_position_cmd(self):
        return commands2.cmd.runOnce(
            lambda: self.armSolenoid.set(DoubleSolenoid.Value.kReverse))\
            .andThen(InstantCommand(lambda: self.armSolenoid2.set(DoubleSolenoid.Value.kReverse))
        )
        
        
    def gripper_open_cmd(self):
        return commands2.cmd.runOnce(
            lambda: self.gripperSolenoid.set(DoubleSolenoid.Value.kForward)
        ) 
        
    def gripper_close_cmd(self):
            return commands2.cmd.runOnce(
            lambda: self.gripperSolenoid.set(DoubleSolenoid.Value.kReverse)
        )