package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class AutoPathPlanningTest extends Command {

    EncoderFollower eLeft, eRight;
    Trajectory tLeft, tRight;
    Paths paths;
    public AutoPathPlanningTest(){
        requires(Robot.kDrivetrain);
        Trajectory.Config configuration = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 1/200, 12, 4, 20);
        Trajectory traj = Pathfinder.generate(Paths.CenterSwitch, configuration);
        paths = new Paths(traj, configuration, 2.16);
        tLeft = paths.getLeft();
        tRight = paths.getRight();
        eLeft = new EncoderFollower(tLeft);
        eRight = new EncoderFollower(tRight);
        eLeft.configureEncoder((int)Robot.kDrivetrain.getEncoderPositions()[0], Constants.PATHFINDER_ENC_TICKS, Constants.PATHFINDER_WHEEL_DIA);
        eRight.configureEncoder((int)Robot.kDrivetrain.getEncoderPositions()[1], Constants.PATHFINDER_ENC_TICKS, Constants.PATHFINDER_WHEEL_DIA);

        eLeft.configurePIDVA(Constants.PATHFINDER_P, Constants.PATHFINDER_I, Constants.PATHFINDER_D, 1/12, Constants.PATHFINDER_A);
        eRight.configurePIDVA(Constants.PATHFINDER_P, Constants.PATHFINDER_I, Constants.PATHFINDER_D, 1/12, Constants.PATHFINDER_A);
    }

    protected void initialize() {


    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double l = eLeft.calculate((int)Robot.kDrivetrain.getEncoderPositions()[0]);
        double r = eLeft.calculate((int)Robot.kDrivetrain.getEncoderPositions()[1]);

        double gyro = Robot.kDrivetrain.getGyroAngle();
        double desired_heading = Pathfinder.r2d(eLeft.getHeading());
        double angleDiff = Pathfinder.boundHalfDegrees(desired_heading-gyro);
        double turn = 0.8 * (-1.0/80.0) * angleDiff;

        Robot.kDrivetrain.setDrivePower(l+turn, r-turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return eLeft.isFinished();

    }

    // Called once after isFinished returns true
    protected void end() {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }
}
