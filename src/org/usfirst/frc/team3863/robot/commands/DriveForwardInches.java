package org.usfirst.frc.team3863.robot.commands;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */   
public class DriveForwardInches extends Command {
	boolean finished = false;
	double distance;
	  
    public DriveForwardInches(double inches) {
    	requires (Robot.kDrivetrain);
    	distance = inches;
    }

    
    protected void initialize() {
    	double ticks = distance / Constants.DRIVE_WHEEL_DIAMETER * Constants.DRIVE_ENCODER_TICKS;
    	Robot.kDrivetrain.setPositionTargetIncrements(-ticks, -ticks*0.75);
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    
    protected boolean isFinished() {    	
    	double err = Robot.kDrivetrain.pidErrorAverage();
    	System.out.println(err);
        return (Math.abs(err)<200 && !(err==0.0));
        
        
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    
    protected void interrupted() {
    }
}
