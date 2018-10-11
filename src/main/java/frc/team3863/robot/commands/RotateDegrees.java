package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;
import jaci.pathfinder.Pathfinder;

/**
 *
 */
public class RotateDegrees extends Command {
    double degree_offset;
    double target_degrees;
    double start_degrees;
    double currentError;
    double lastError;
    double IAccum;
    double errorTolerance;

    public RotateDegrees(double degrees) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kDrivetrain);
        degree_offset = degrees;
        errorTolerance = .5;
    }

    public RotateDegrees(double degrees, double errorTolerance) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kDrivetrain);
        degree_offset = degrees;
        this.errorTolerance = errorTolerance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        start_degrees = Robot.kDrivetrain.getOdometry().getTheta();
        Robot.kDrivetrain.setTransmissionLow();
        //Invert for comp robot gyro mounting?
        target_degrees = Pathfinder.boundHalfDegrees(start_degrees + degree_offset);
        System.out.println("Rotating from " + start_degrees + " to " + target_degrees);
        currentError = target_degrees - Math.toDegrees(Robot.kDrivetrain.getOdometry().getTheta());
        lastError = currentError;
        IAccum = 0;
    }


    protected void execute() {
        currentError = target_degrees - Math.toDegrees(Robot.kDrivetrain.getOdometry().getTheta());
        currentError = Pathfinder.boundHalfDegrees(currentError);
        double setpoint = currentError * Constants.DRIVE_ROTATE_P + IAccum * Constants.DRIVE_ROTATE_I + Constants.DRIVE_ROTATE_D * (lastError - currentError);
        double left =  -setpoint;
        double right = setpoint;

        left = clamp(left, -1, 1);
        right = clamp(right, -1, 1);
        System.out.println("Error: " + currentError + " Left Speed: " + left + " Right Speed: " + right);
        Robot.kDrivetrain.setDrivePower(left, right);
        //Robot.kDrivetrain.setDrivePower(left, right);
        lastError = currentError;
        if(currentError > 0){
            IAccum++;
        } else if (currentError < 0){
            IAccum--;
        }else{
            IAccum +=0;
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(currentError) < errorTolerance;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.kDrivetrain.setFPS(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    private double clamp(double value, double min, double max){
        if(value > max){
            return max;
        } else if(value < min){
            return min;
        }
        else
            return value;
    }

}
