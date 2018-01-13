package org.usfirst.frc.team3863.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team3863.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
/**
 Controls the four CANTalons dedicated to the Drivetrain
 */
public class Drivetrain extends Subsystem {

	WPI_TalonSRX talonLeftA = new WPI_TalonSRX(RobotMap.TALON_DRIVE_LEFTA_ID);
	WPI_TalonSRX talonLeftB = new WPI_TalonSRX(RobotMap.TALON_DRIVE_LEFTB_ID);
	WPI_TalonSRX talonRightA = new WPI_TalonSRX(RobotMap.TALON_DRIVE_RIGHTA_ID);
	WPI_TalonSRX talonRightB = new WPI_TalonSRX(RobotMap.TALON_DRIVE_RIGHTB_ID);
	
	DoubleSolenoid transmissiom_solenoid = new DoubleSolenoid(RobotMap.PCM_TRANSMISSION_LOW, RobotMap.PCM_TRANSMISSION_HIGH);
	
	ControlMode mode = ControlMode.PercentOutput;
	
	public boolean transmission_in_low = true;
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setDrivePower(double left, double right) {
    	//Set the left and right motor power 
    	talonLeftA.set(mode, left);
    	talonLeftB.set(mode,left);
    	
    	talonRightA.set(mode, right);
    	talonRightB.set(mode, right);
    }
    
    public void setTransmissionLow() {
    	System.out.println("Transmission in Low Gear");
    	transmission_in_low = true;
    	transmissiom_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
	public void setTransmissionHigh() {
		System.out.println("Transmission in High Gear");
		transmission_in_low = false;
		transmissiom_solenoid.set(DoubleSolenoid.Value.kReverse);
	}
    
}

