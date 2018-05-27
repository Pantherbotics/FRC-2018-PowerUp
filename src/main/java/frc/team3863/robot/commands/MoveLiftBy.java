package frc.team3863.robot.commands;

import frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveLiftBy extends Command {
	int pos_increment;
    public MoveLiftBy(int position_increment) {
        requires(Robot.kElevator);
        pos_increment = position_increment;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.kElevator.setTargetPosition(Robot.kElevator.target + pos_increment);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
