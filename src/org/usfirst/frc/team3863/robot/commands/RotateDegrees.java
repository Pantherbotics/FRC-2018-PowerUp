package org.usfirst.frc.team3863.robot.commands;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RotateDegrees extends Command {
	double degree_offset;
	double target_degrees;
	double start_degrees;

    public RotateDegrees(double degrees) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kDrivetrain);
        degree_offset = degrees;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	start_degrees = Robot.kDrivetrain.getGyroAngle();
    	target_degrees = start_degrees + degree_offset;
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double error = target_degrees - Robot.kDrivetrain.getGyroAngle();
    	double left = error * Constants.DRIVE_ROTATE_P;
    	double right = error * Constants.DRIVE_ROTATE_P * -1;
    	Robot.kDrivetrain.setPositionTargetIncrements(left, right);
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
