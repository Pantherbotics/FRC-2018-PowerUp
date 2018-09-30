package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Robot;

/**
 *
 */
public class ElevatorSetpoint extends Command {
    int set;
    boolean isDelayed;
    double startTime, delay;
    public ElevatorSetpoint(int preset) {
        set = preset;
        isDelayed = false;
        requires(Robot.kElevator);
    }

    public ElevatorSetpoint(int preset, double delaySeconds){
        set = preset;
        requires(Robot.kElevator);
        isDelayed = true;
        delay = delaySeconds;
        startTime = System.nanoTime()/1E9;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if(isDelayed) {
            System.out.println("Has " + delay + " passed? " + (Math.abs(((System.nanoTime()/1E9)-startTime) - delay) < 0.1));
            if(Math.abs(((System.nanoTime()/1E9)-startTime) - delay) < 0.1)
                Robot.kElevator.goToPreset(set);
        }else
            Robot.kElevator.goToPreset(set);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(!isDelayed)
            return true;
        else{
            return Math.abs(((System.nanoTime()/1E9)-startTime) - delay) < 0.1;
        }

    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
