package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;
import frc.team3863.robot.util.RamseteFollower;
import jaci.pathfinder.Trajectory;

public class AutoPathFollower extends Command  {

    private double start;
    private RamseteFollower follower;
    private Notifier trackingThread;
    private volatile boolean isFinished;

    public AutoPathFollower(Trajectory traj) throws NullPointerException{
        requires(Robot.kDrivetrain);
        System.out.println(traj.length());
        follower = new RamseteFollower(Constants.WHEEL_BASE, traj);
        System.out.println("Created new Follower");
        trackingThread = new Notifier(() -> {
            if(!isFinished){
            isFinished = follower.isFinished();
            follower.setOdometry(Robot.kDrivetrain.getOdometry());
            System.out.println("Getting next DriveSignal");
            System.out.println(follower.getNextDriveSignal().getLeft() + ", " + follower.getNextDriveSignal().getRight());
            Robot.kDrivetrain.setFPS(follower.getNextDriveSignal().getLeft(), follower.getNextDriveSignal().getRight());
            }
        });
    }

    protected void initialize() {
        Robot.kDrivetrain.setTransmissionHigh();
        start = System.nanoTime();
        trackingThread.startPeriodic(0.015);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return follower.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
        trackingThread.stop();
        System.out.println("Finished!");
        System.out.println("Time to completion: " + ((System.nanoTime()-start)/1000000));
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }
}
