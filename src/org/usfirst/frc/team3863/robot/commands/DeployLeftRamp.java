package org.usfirst.frc.team3863.robot.commands;

import org.usfirst.frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DeployLeftRamp extends Command {
	boolean can_retract;

    public DeployLeftRamp(boolean retractable) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kRamps);
        can_retract = retractable;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.kRamps.is_left_ramp_deployed && !can_retract) {
    		Robot.kRamps.liftLeftRamp();
    	}else if (!can_retract){
    		Robot.kRamps.deployLeftRamp();
    	}else {
    		Robot.kRamps.liftLeftRamp();
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !can_retract;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.kRamps.retractLeftRamp();
    }
}
