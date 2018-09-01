package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;
import frc.team3863.robot.util.RamseteFollower;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class AutoPathFollower extends Command {

    private RamseteFollower follower;

    public AutoPathFollower(Trajectory traj) {
        requires(Robot.kDrivetrain);
        System.out.println(traj.length());
        follower = new RamseteFollower(Constants.WHEEL_BASE, traj);
        System.out.println("Created new Follower");
    }

    protected void initialize() {
        Robot.kDrivetrain.setTransmissionHigh();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        follower.setOdometry(Robot.kDrivetrain.getOdometry());
        System.out.println("Getting next DriveSignal");
        System.out.println(follower.getNextDriveSignal().getLeft() + ", " + follower.getNextDriveSignal().getRight());
        Robot.kDrivetrain.setFPS(follower.getNextDriveSignal().getLeft(), follower.getNextDriveSignal().getRight());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return follower.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }
}
