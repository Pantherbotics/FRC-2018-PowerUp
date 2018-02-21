package org.usfirst.frc.team3863.robot.subsystems;

import org.usfirst.frc.team3863.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Ramps extends Subsystem {

	DoubleSolenoid left_ramp_solenoid = new DoubleSolenoid(RobotMap.PCM_LEFT_RAMP_DEPLOY, RobotMap.PCM_LEFT_RAMP_RETRACT);
	DoubleSolenoid right_ramp_solenoid = new DoubleSolenoid(RobotMap.PCM_RIGHT_RAMP_DEPLOY, RobotMap.PCM_RIGHT_RAMP_RETRACT);

	Servo left_servo = new Servo(RobotMap.PWM_LEFT_SERVO);
	Servo right_servo = new Servo(RobotMap.PWM_RIGHT_SERVO);
	
	public boolean is_left_ramp_deployed = false;
	public boolean is_right_ramp_deployed = false;
	
	
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void init() {
    	left_servo.setAngle(90);
    	right_servo.setAngle(90);
    	retractLeftRamp();
    	retractRightRamp();
    };
    
    
    public void liftLeftRamp() {
    	left_ramp_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void liftRightRamp() {
    	right_ramp_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void retractLeftRamp() {
    	left_ramp_solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void retractRightRamp() {
    	right_ramp_solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void deployLeftRamp() {
    	left_servo.setAngle(170);
    	is_left_ramp_deployed = true;
    }
    
    public void deployRightRamp() {
    	right_servo.setAngle(170);
    	is_left_ramp_deployed = true;
    }
}

