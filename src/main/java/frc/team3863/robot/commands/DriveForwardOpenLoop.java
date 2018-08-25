package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Robot;

/**
 *
 */
public class DriveForwardOpenLoop extends Command {
    double loops;
    boolean done = false;
    double counter = 0;

    public DriveForwardOpenLoop(double loopsToRun) {
        requires(Robot.kDrivetrain);
        loops = loopsToRun;
    }


    protected void initialize() {
        System.out.println("Driving forward " + loops + " iters");
        //Robot.kDrivetrain.zeroEncoderPositions();
        //double currentPos[] = Robot.kDrivetrain.getEncoderPositions();
        done = false;
        counter = 0;
        Robot.kDrivetrain.setTransmissionLow();
        Robot.kDrivetrain.setDrivePower(-0.75, -0.75);
        //targetPos = ((currentPos[0] + currentPos[1]) /2) - ticks;

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        //Black voodoo magic to calculate PID AND drive error
        //double currentPos[] = Robot.kDrivetrain.getEncoderPositions();
        //double perr = ((currentPos[0] + currentPos[1]) /2) - targetPos;
        System.out.println("iter:" + (counter));
        done = (counter > loops);
        if (done) {
            Robot.kDrivetrain.setDrivePower(0, 0);
        }
        counter += 1;
    }


    protected boolean isFinished() {
        return done;
    }

    // Called once after isFinished returns true
    protected void end() {

    }

    // Called when another command which requires one or more of the same

    protected void interrupted() {
    }
}
