
# package com.ctre.phoenix6.mechanisms.swerve;
import wpilib
import phoenix6
import wpimath.units
import constants
import wpilib.simulation

from phoenix6.hardware import Pigeon2, cancoder
from phoenix6.sim import CANcoderSimState, ChassisReference, Pigeon2SimState, TalonFXSimState 
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry, \
ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d
from phoenix6 import unmanaged


from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor

from phoenix6.hardware import TalonFXSimState, TalonFX

import math
#import edu.wpi.first.wpilibj.simulation.DCMotorSim;

class SimSwerveModule: 
    def __init__(self, steerGearing, steerInertia, steerFrictionVoltage, steerMotorInverted,
                 driveGearing, driveInertia, driveFrictionVoltage, driveMotorInverted):

        self.SteerMotorSim = DCMotorSim(DCMotor.krakenX60(1), steerGearing, steerInertia)
        self.DriveMotorSim = DCMotorSim(DCMotor.krakenX60(1), driveGearing, driveInertia)
        self.SteerGearing = steerGearing
        self.DriveGearing = driveGearing
        self.SteerFrictionVoltage = steerFrictionVoltage
        self.DriveFrictionVoltage = driveFrictionVoltage
        self.SteerMotorInverted = steerMotorInverted
        self.DriveMotorInverted = driveMotorInverted
    
    def update(self):
        self.SteerMotorSim = modulesToApply().getSteerMotorSim().getSimState()
        self.DriveMotorSim = modulesToApply().getDriveMotorSim().getSimState()
        self.CANcoderSim = modulesToApply().getCANcoderSim().getSimState()

        
        self.SteerMotorSim.Orientaiton = self.SteerMotorInverted(ChassisReference.Clockwise_Positive, ChassisReference.CounterClockwise_Positive)
        self.DriveMotorSim.Orientation = self.DriveMotorInverted(ChassisReference.Clockwise_Positive, ChassisReference.CounterClockwise_Positive)
        
        if self.SteerMotorInverted:
            steer

        #steerMotor.setSupplyVoltage(supplyVoltage)
        #driveMotor.setSupplyVoltage(supplyVoltage)
        #cancoder.setSupplyVoltage(supplyVoltage)

        self.SteerMotorSim.set_supply_voltage(12.0)
        self.DriveMotorSim.set_supply_voltage(12.0)
        self.CANcoderSim.set_supply_voltage(12.0)


        #self.SteerMotorSim.setInputVoltage(addFriction(steerMotor.getMotorVoltage(), self.SteerFrictionVoltage))
        #self.DriveMotorSim.setInputVoltage(addFriction(driveMotor.getMotorVoltage(), self.DriveFrictionVoltage))

    def update_talonFX(self, motor:TalonFX, motor_sim:DCMotorSim, tm_diff):

        motor_sim.setInputVoltage(motor.sim_state.motor_voltage)
        
        motor_sim.update(tm_diff)

        self.DriveMotorSim(self.SteerMotor.update(dtSeconds))
        
        self.DriveMotorSim.sim_state.set_raw_rotor_position(motor_sim.getAngularPosition() * constants.kSteerGearReduction)
        self.DriveMotorSim.set_rotor_velocity(motor_sim.getAngularVelocity() * constants.kDriveGearReduction)

        # CANcoders see the mechanism, so don't account for the steer gearing */
        cancoder.set_raw_rotor_position(self.SteerMotorSim.getAngularPosition())
        cancoder.set_rotor_velocity(self.SteerMotorSim.getAngularVelocity()) #getAngularVelocityRPM() / 60.0)

        motor.sim_state.set_raw_rotor_position(motor_sim.getAngularPosition())
        motor.sim_state.set_rotor_velocity(motor_sim.getAngularVelocity())
        #states() = modulesToApply().getCurrentState()
        
        
    
    
class SimSwerveDrivetrain:

    def __init__(self, pigeon):
    #                         Pigeon2 pigeon,
    #                           SwerveDrivetrainConstants driveConstants,
    #                           SwerveModuleConstants ... moduleConstants) {
    #    PigeonSim = pigeon.getSimState();
        self._pigeonSim = Pigeon2SimState()
        self._flsim = SimSwerveModule(False, False) # steer not inverted, drive not inverted
        self._frsim = SimSwerveModule(True, False) # steer inverted, drive not inverted
        self._blsim = SimSwerveModule(False, True) # steer not inverted, drive inverted
        self._brsim = SimSwerveModule(True, True) # steer inverted, drive inverted
        

     # Update this simulation for the time duration.
     #<p>
     # This performs a simulation update on all the simulated devices
     # @param dtSeconds The time delta between this update and the previous update
     # @param supplyVoltage The voltage as seen at the motor controllers
     # @param modulesToApply What modules to apply the update to
     
    def update(self, dtSeconds, supplyVoltage, SwerveModule, modulesToApply):


        SwerveModuleState states = SwerveModuleState[ModuleCount]
        #/* Update our sim devices */

        self._flsim.update()
        self._frsim.update()
        self._blsim.update()
        self._brsim.update()
            

        DoubleAngleChange = Kinem.toChassisSpeeds(states).omegaRadiansPerSecond * dtSeconds
        LastAngle = LastAngle.plus(Rotation2d.fromRadians(angleChange))
        self.phoenix6_sim_state.setRawYaw(LastAngle.get_degree)
    

    
     # Applies the effects of friction to dampen the motor voltage.
     # @param motorVoltage Voltage output by the motor
     # @param frictionVoltage Voltage required to overcome friction
     # @return Friction-dampened motor voltage
    
    def  addFriction(self, motorVoltage, frictionVoltage):
        if (Math.abs(motorVoltage) < frictionVoltage):
            motorVoltage = 0.0
        elif (motorVoltage > 0.0):
            motorVoltage -= frictionVoltage
        else
            motorVoltage += frictionVoltage

        return motorVoltage



























































