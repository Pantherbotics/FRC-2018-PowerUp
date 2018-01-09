package org.usfirst.frc.team3863.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 Controls the four CANTalons dedicated to the Drivetrain
 */
public class Drivetrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setDrivePower(float left, float right) {
    	//Set the left and right motor power 
    	//TODO: Actually implement this with CANTalons
    	System.out.println("LeftMotorPower: " + left + " RightMotorPower: %s" + right);
    }
}

