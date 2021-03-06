package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Robot;

/**
 *
 */
public class DeployIntake extends Command {
    int counter = 0;

    public DeployIntake() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        counter = 0;
        //Robot.kIntake.setIntakeWheelPower(-1);
        //Robot.kIntake.openClaw();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        counter += 1;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return counter > 25;
    }

    // Called once after isFinished returns true
    protected void end() {
        //Robot.kIntake.closeClaw();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        //Robot.kIntake.setIntakeWheelPower(-1);
    }
}
