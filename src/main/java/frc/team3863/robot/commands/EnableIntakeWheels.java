package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;

/**
 *
 */
public class EnableIntakeWheels extends Command {
    boolean reverse;
    boolean canfin = false;
    double timeout, inc;
    public EnableIntakeWheels(boolean reverse_direction) {
        reverse = reverse_direction;
        //requires(Robot.kIntake);
        this.timeout = -1;
    }

    public EnableIntakeWheels(boolean reverse_direction, double timeout) {
        reverse = reverse_direction;
        //requires(Robot.kIntake);
        this.timeout = timeout;
        inc = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        canfin = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (reverse) {
            Robot.kIntake.setIntakeWheelPower(-Constants.INTAKE_MOTOR_POWER);
        } else {
            Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_MOTOR_POWER);
        }

        inc+=0.02;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return canfin || inc >= timeout;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_IDLE_POWER);
        canfin = true;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_IDLE_POWER);
        canfin = true;
    }
}
