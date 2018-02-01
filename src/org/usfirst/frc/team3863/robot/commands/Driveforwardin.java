package org.usfirst.frc.team3863.robot.commands;

import org.usfirst.frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */   
public class Driveforwardin extends Command {

	
	  double distance;
	  
    public Driveforwardin(double inches) {
    	requires (Robot.kDrivetrain);
    	
    	distance = inches;
    	
    	
    	
    	
    	
        
    }

    
    protected void initialize() {
    	
    	
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double ticks = distance * 18.850/100;
    	Robot.kDrivetrain.setPositionTargetIncrements(ticks, ticks);
    	
    	
    	
    	
    	
    }

    
    protected boolean isFinished() {    	
    	
        return (Math.abs(Robot.kDrivetrain.piderroraverage())<20);
        
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    
    protected void interrupted() {
    }
}
