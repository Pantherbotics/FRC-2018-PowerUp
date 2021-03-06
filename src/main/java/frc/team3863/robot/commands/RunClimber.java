package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Robot;

/**
 *
 */
public class RunClimber extends Command {

    boolean fin;

    public RunClimber() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.kClimber);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        fin = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.kClimber.setWinchPower(1.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return fin;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.kClimber.setWinchPower(0);
        fin = true;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.kClimber.setWinchPower(0);
        fin = true;
    }
}
