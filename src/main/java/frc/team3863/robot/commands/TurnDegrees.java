package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.util.Units;

import static frc.team3863.robot.Robot.kDrivetrain;

/**
 *
 */
public class TurnDegrees extends Command {

    private double degrees;
    private double startAngle, currentAngle;
    private double acceptableError;

    public TurnDegrees(double degrees, double acceptableError) {
        // Use requires() here to declare subsystem dependencies
        requires(kDrivetrain);
        this.degrees = degrees;
        this.acceptableError = acceptableError;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        double arcLength = (Constants.WHEEL_BASE/2.0) * Math.toRadians(degrees);
        double increment = Units.FeetToTalonNative(arcLength);
        startAngle = kDrivetrain.getOdometry().getTheta();
        kDrivetrain.setPositionTargetIncrements(-increment, increment);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        currentAngle = kDrivetrain.getOdometry().getTheta();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(Math.toDegrees((startAngle + Math.toRadians(degrees)) - currentAngle)) < acceptableError;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
