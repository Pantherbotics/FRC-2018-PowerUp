package org.usfirst.frc.team3863.robot.subsystems;

import org.usfirst.frc.team3863.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {

	public WPI_TalonSRX leftIntakeTalon = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_TALON_ID);
	public WPI_TalonSRX rightIntakeTalon = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_TALON_ID);
	//public WPI_TalonSRX pivotTalon = new WPI_TalonSRX(RobotMap.INTAKE_PIVOT_TALON_ID);
	
	DoubleSolenoid claw_solenoid = new DoubleSolenoid(RobotMap.PCM_INTAKE_RETRACT, RobotMap.PCM_INTAKE_EXTEND);
	
	int pid_id = 0;
	public int timeout_ms = 0;

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
}

