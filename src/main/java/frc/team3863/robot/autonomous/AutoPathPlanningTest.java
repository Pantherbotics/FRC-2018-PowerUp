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
        Trajectory.Config configuration = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 1/200, 12, 4, 20);
        Trajectory traj = Pathfinder.generate(Paths.CenterSwitch, configuration);
        follower = new RamseteFollower(Robot.kDrivetrain.getTalons()[0], Robot.kDrivetrain.getTalons()[1], Constants.WHEEL_BASE, traj, Robot.kDrivetrain.getGyro());
    }

    protected void initialize() {
        Robot.kDrivetrain.setTransmissionHigh();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
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
