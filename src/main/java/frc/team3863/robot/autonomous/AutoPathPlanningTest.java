package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import frc.team3863.robot.util.RamseteFollower;

public class AutoPathPlanningTest extends Command {

    EncoderFollower eLeft, eRight;
    Trajectory tLeft, tRight;
    Paths paths;
    RamseteFollower follower;
    public AutoPathPlanningTest(){
        requires(Robot.kDrivetrain);
        Trajectory traj = Paths.getLastTraj();
        System.out.println(traj.length());
        follower = new RamseteFollower(Robot.kDrivetrain.getTalons()[0], Robot.kDrivetrain.getTalons()[1], Constants.WHEEL_BASE, traj, Robot.kDrivetrain.getGyro());
        System.out.println("Created new Follower");
    }

    protected void initialize() {
        Robot.kDrivetrain.setTransmissionHigh();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	follower.setOdometry(Robot.kDrivetrain.getOdometry()[0], Robot.kDrivetrain.getOdometry()[1]);
    	System.out.println("executing!");
    	System.out.println(follower.getNextWheelCommand().getLeft() + ", " + follower.getNextWheelCommand().getRight());
        Robot.kDrivetrain.setDrivePower(follower.getNextWheelCommand().getLeft(), follower.getNextWheelCommand().getRight());
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
