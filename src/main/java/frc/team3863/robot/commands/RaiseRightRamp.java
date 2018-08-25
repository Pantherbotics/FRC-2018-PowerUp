package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Robot;

/**
 *
 */
public class RaiseRightRamp extends Command {
    public RaiseRightRamp() {
        // Use requires() here to declare subsystem dependencies
        //requires(Robot.kRamps);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (Robot.kRamps.is_right_ramp_deployed) {
            Robot.kRamps.liftRightRamp();
            System.out.println("Right Ramp Raised");
        } else {
            System.out.println("Right Ramp Not Raised");
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
