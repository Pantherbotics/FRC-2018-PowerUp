package frc.team3863.robot.commands;

import frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * this code sucks - rewrite at some point!
 */
public class RaiseLeftRamp extends Command {

    public RaiseLeftRamp() {
        // Use requires() here to declare subsystem dependencies
        //requires(Robot.kRamps);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.kRamps.is_left_ramp_deployed) {
    		Robot.kRamps.liftLeftRamp();
    		System.out.println("Left Ramp Raised");
    	}else {
    		System.out.println("Left Ramp Not Raised");
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
