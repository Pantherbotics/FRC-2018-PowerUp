package org.usfirst.frc.team3863.robot.commands;

import org.usfirst.frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ZeroLift extends Command {

    public ZeroLift() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kElevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.kElevator.setMotorPower(-0.5);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Robot.kElevator.isLiftLowered());
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.kElevator.setMotorPower(0);
    	Robot.kElevator.zeroEncoder();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
