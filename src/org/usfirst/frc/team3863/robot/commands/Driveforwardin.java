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
    	System.out.println("1234");
    	double ticks = distance / 18.850 * 400;
    	Robot.kDrivetrain.setPositionTargetIncrements(ticks, ticks);
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    
    protected boolean isFinished() {    	
    	System.out.println("isFinished: " + (Math.abs(Robot.kDrivetrain.piderroraverage())<20) + " avgerr: " + Robot.kDrivetrain.piderroraverage());
        //return (Math.abs(Robot.kDrivetrain.piderroraverage())<20);
    	return false;
        
        
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    
    protected void interrupted() {
    }
}
