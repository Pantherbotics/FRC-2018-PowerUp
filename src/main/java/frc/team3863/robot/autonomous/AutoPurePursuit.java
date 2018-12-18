package frc.team3863.robot.autonomous;

import frc.team3863.robot.util.PurePursuit;
import frc.team3863.robot.Robot;
import frc.team3863.robot.Constants;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import jaci.pathfinder.Trajectory;

public class AutoPurePursuit extends Command{

    private PurePursuit pursuit;
    private Notifier tracking;
    private volatile boolean isFinished;
    private volatile Timer time;
    private double now;

    public AutoPurePursuit(Trajectory traj){
        requires(Robot.kDrivetrain);
        pursuit = new PurePursuit(Constants.WHEEL_BASE, traj);
        time.start();

        tracking = new Notifier(() -> {
            if(!isFinished){
                isFinished = pursuit.isFinished();
                now = time.get();
                pursuit.setOdometry(Robot.kDrivetrain.getOdometry());
                System.out.println("Left: " + pursuit.getNextDriveSignal(now).getLeft() + "\nRight: " + pursuit.getNextDriveSignal(now).getRight() + "\n");
                Robot.kDrivetrain.setFPS(pursuit.getNextDriveSignal(now).getLeft(), pursuit.getNextDriveSignal(now).getRight());
            }
        });
        time.stop();
    }

    protected boolean isFinished(){
        return pursuit.isFinished();
    }
}