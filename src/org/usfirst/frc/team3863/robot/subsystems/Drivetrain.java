package org.usfirst.frc.team3863.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team3863.robot.RobotMap;

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
	
	ControlMode mode = ControlMode.PercentOutput;
	
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
    
}

