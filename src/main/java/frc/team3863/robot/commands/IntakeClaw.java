package frc.team3863.robot.commands;

import frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeClaw extends Command {
    boolean extend;
    public IntakeClaw(boolean open) {
    	extend = open;
        requires(Robot.kIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (extend) {
    		Robot.kIntake.openClaw();
    	}else {
    		Robot.kIntake.closeClaw();
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
