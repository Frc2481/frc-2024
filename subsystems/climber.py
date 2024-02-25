import wpilib
from wpilib import DoubleSolenoid
import constants
from commands2 import *
from commands2.cmd import *

class ClimberSubsystem(Subsystem):
    
    def __init__(self):
        
        self.climberSolenoid = DoubleSolenoid(
            # constants.kGripperSolenoidModule,\[]
            
            moduleType = wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel = constants.kClimberDoubleSolenoidForwardPort,
            reverseChannel = constants.kClimberDoubleSolenoidReversePort
         )        
    # Climber up
    def kill_the_beast_cmd(self):
        return runOnce(
            lambda: self.climberSolenoid.set(DoubleSolenoid.Value.kReverse)
        ) 
    
    # Climber down    
    def Batman_grapling_hook_cmd(self):
        return runOnce(
            lambda: self.climberSolenoid.set(DoubleSolenoid.Value.kForward)
        )