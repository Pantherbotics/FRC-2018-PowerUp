package org.usfirst.frc.team3863.robot.subsystems;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {

	public WPI_TalonSRX leftIntakeTalon = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_TALON_ID);
	public WPI_TalonSRX rightIntakeTalon = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_TALON_ID);
	Counter ultrasonicCounter = new Counter(RobotMap.SENSOR_INTAKE_ULTRASONIC);
	
	DoubleSolenoid claw_solenoid = new DoubleSolenoid(RobotMap.PCM_INTAKE_RETRACT, RobotMap.PCM_INTAKE_EXTEND);
	
	int pid_id = 0;
	public int timeout_ms = 0;
	
	public void init() {
		leftIntakeTalon.configContinuousCurrentLimit(Constants.INTAKE_CURRENT_LIMIT, timeout_ms);
		leftIntakeTalon.enableCurrentLimit(true);
		rightIntakeTalon.configContinuousCurrentLimit(Constants.INTAKE_CURRENT_LIMIT, timeout_ms);
		rightIntakeTalon.enableCurrentLimit(true);
		
		ultrasonicCounter.setSemiPeriodMode(false);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setIntakeWheelPower(double power) {
    	leftIntakeTalon.set(ControlMode.PercentOutput, power);
    	rightIntakeTalon.set(ControlMode.PercentOutput, -power);
    }
    
    public void closeClaw() {
    	claw_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void openClaw() {
    	claw_solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public double getRawDistance() {
    	//147 uS per inch
    	return (ultrasonicCounter.getPeriod() * 3 / -0.000147) + 992; 
    }
    
    public boolean isCubeInIntake() {
    	return (getRawDistance() < 1.0);
    	
    }
    
    public boolean testMotorCurrentThreshold(double amps) {
    	return (leftIntakeTalon.getOutputCurrent()+rightIntakeTalon.getOutputCurrent()/2) > amps;
    }
    
    public boolean isCubeInfrontOfIntake() {
    	double dist = getRawDistance();
    	return (dist < Constants.AUTO_INTAKE_MAX_DIST && dist > 1.0);
    }
}

