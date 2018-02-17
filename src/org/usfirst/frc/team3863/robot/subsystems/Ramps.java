package org.usfirst.frc.team3863.robot.subsystems;

import org.usfirst.frc.team3863.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Ramps extends Subsystem {

	DoubleSolenoid left_ramp_solenoid = new DoubleSolenoid(RobotMap.PCM_LEFT_RAMP_DEPLOY, RobotMap.PCM_LEFT_RAMP_RETRACT);
	DoubleSolenoid right_ramp_solenoid = new DoubleSolenoid(RobotMap.PCM_RIGHT_RAMP_DEPLOY, RobotMap.PCM_RIGHT_RAMP_RETRACT);

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void init() {
    	
    }
    
    public void deployLeftRamp() {
    	left_ramp_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void deployRightRamp() {
    	right_ramp_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void retractLeftRamp() {
    	left_ramp_solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void retractRightRamp() {
    	right_ramp_solenoid.set(DoubleSolenoid.Value.kReverse);
    }
}

