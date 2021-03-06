package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;

/**
 *
 */
public class RotateDegrees extends Command {
    double degree_offset;
    double target_degrees;
    double start_degrees;
    double error;

    public RotateDegrees(double degrees) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kDrivetrain);
        degree_offset = degrees;
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        start_degrees = Robot.kDrivetrain.getOdometry().getTheta();
        Robot.kDrivetrain.setTransmissionLow();
        //Invert for comp robot gyro mounting?
        target_degrees = start_degrees + degree_offset;
        System.out.println("Rotating from " + start_degrees + " to " + target_degrees);
    }

    // Called repeatedly when this Command is scheduled to run
    // TODO: Direct motor power instead of velocity (elim. encoder feedback)
    // three step: < 33%, lower gain
    //             < 66%, full gain
    //             < 100%, slower gain      
    protected void execute() {
        error = target_degrees - Robot.kDrivetrain.getOdometry().getTheta();
        double left = error * Constants.DRIVE_ROTATE_P * -1;
        double right = error * Constants.DRIVE_ROTATE_P;
        System.out.println("" + error + " " + left + " " + right);
        Robot.kDrivetrain.setFPS(left * 20, right * 20);
        //Robot.kDrivetrain.setDrivePower(left, right);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(error) < 6;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.kDrivetrain.setFPS(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
