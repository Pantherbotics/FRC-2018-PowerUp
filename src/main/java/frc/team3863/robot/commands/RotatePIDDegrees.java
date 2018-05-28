package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RotatePIDDegrees extends Command {
    double degree_offset;
    double target_degrees;
    double error;
    PIDController pidController;

    public RotatePIDDegrees(double degrees, double tolerance) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kDrivetrain);
        degree_offset = degrees;
        target_degrees = Robot.kDrivetrain.getGyroAngle() + degree_offset;
        pidController = new PIDController(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D, Constants.TURN_F, Robot.kDrivetrain, Robot.kDrivetrain);
        pidController.setAbsoluteTolerance(tolerance);
        pidController.setSetpoint(target_degrees);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        pidController.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    // TODO: Direct motor power instead of velocity (elim. encoder feedback)
    // three step: < 33%, lower gain
    //             < 66%, full gain
    //             < 100%, slower gain
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return pidController.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
        pidController.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
