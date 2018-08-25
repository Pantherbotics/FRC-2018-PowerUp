package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Robot;

/**
 *
 */
public class SetHookDegrees extends Command {
    private double setpoint;

    public SetHookDegrees(double degrees) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.kClimber);
        setpoint = ((degrees % 360) / 360) * 4096;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.kClimber.setArmPos(setpoint);
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
