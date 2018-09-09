package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;
import frc.team3863.robot.util.RamseteFollower;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class AutoPathTest extends Command  {

    private Trajectory traj;
    private int idx;

    public AutoPathTest(Trajectory traj) throws NullPointerException{
        this.traj = traj;
        requires(Robot.kDrivetrain);
        System.out.println(traj.length());
        System.out.println("Created new Path tester!");
        idx=0;
    }

    protected void initialize() {
        Trajectory.Segment seg = traj.get(0);
        Robot.kDrivetrain.setTransmissionHigh();
        Robot.kDrivetrain.setOdometry(seg.x, seg.y, seg.heading);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double w;
        if(idx < traj.length()-2 && idx > 0) {
            w = (traj.get(idx + 1).heading - traj.get(idx).heading) / (traj.get(idx).dt);
        } else{
            w = 0;
        }
        double v = traj.get(idx).velocity;
        double left = (-Constants.WHEEL_BASE * w) / 2 + v;  //do math to convert angular velocity + linear velocity into left and right wheel speeds (fps)
        double right = (+Constants.WHEEL_BASE * w) / 2 + v;
        Robot.kDrivetrain.setFPS(left, right);
        idx++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return idx == traj.length()-1;
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("Finished!");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }
}
