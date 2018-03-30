package org.usfirst.frc.team3863.robot.commands;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */   
public class DriveForwardInches extends Command {
	double distance;
	boolean done = false;
	double counter = 0;
	  
    public DriveForwardInches(double inches) {
    	requires (Robot.kDrivetrain);
    	distance = inches;
    }

    
    protected void initialize() {
    	System.out.println("Driving forward "+ distance + " inches");
    	Robot.kDrivetrain.zeroEncoderPositions();
    	//double currentPos[] = Robot.kDrivetrain.getEncoderPositions();
    	Robot.kDrivetrain.setTransmissionHigh();
    	done = false;
    	counter = 0;
    	double ticks = distance / Constants.DRIVE_WHEEL_DIAMETER * Constants.DRIVE_ENCODER_TICKS;
    	System.out.println("ticks: " + ticks);
    	Robot.kDrivetrain.setPositionTargetIncrements(-ticks, -ticks);
    	//targetPos = ((currentPos[0] + currentPos[1]) /2) - ticks;
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Black voodoo magic to calculate PID AND drive error
    	double err = Robot.kDrivetrain.pidErrorAverage();
    	double[] vels = Robot.kDrivetrain.getEncoderVelocities();
    	//double currentPos[] = Robot.kDrivetrain.getEncoderPositions();
    	//double perr = ((currentPos[0] + currentPos[1]) /2) - targetPos;
    	System.out.println("err: "+err+"vLeft: "+vels[0]+" vRight: "+vels[1]);
    	done = (Math.abs(err)<500 && !(err==0.0) && counter > 100);
    	counter += 1;
    }

    
    protected boolean isFinished() {    	
    	return done;
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    
    protected void interrupted() {
    }
}
