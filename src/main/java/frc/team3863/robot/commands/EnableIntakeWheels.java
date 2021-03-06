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

    public EnableIntakeWheels(boolean reverse_direction) {
        reverse = reverse_direction;
        //requires(Robot.kIntake);
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
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return canfin;
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
