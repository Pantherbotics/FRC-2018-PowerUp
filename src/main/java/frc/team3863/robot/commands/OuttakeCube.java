package frc.team3863.robot.commands;

import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class OuttakeCube extends Command {
	int counter = 0;
    public OuttakeCube() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	counter = 0;
    	System.out.println("Outtaking Cube");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_MOTOR_POWER);
    	//Robot.kIntake.openClaw();
    	counter += 1;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return counter > 40;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.kIntake.setIntakeWheelPower(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
