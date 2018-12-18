/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;

import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;
import frc.team3863.robot.util.PurePursuit;
import frc.team3863.robot.util.Odometry;

import jaci.pathfinder.Trajectory;

public class AutoPurePursuit extends Command {

  private PurePursuit pursuit;
  private Timer time;
  private double now;

  public AutoPurePursuit(Trajectory traj) {
    requires(Robot.kDrivetrain);
    pursuit = new PurePursuit(Constants.WHEEL_BASE, traj);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.kDrivetrain.setOdometry(pursuit.getInitOdometry());
    time.start();
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    Robot.kDrivetrain.setTransmissionHigh();
    pursuit.setOdometry(Robot.kDrivetrain.getOdometry());
    now = time.get();
    System.out.println("Left: " + pursuit.getNextDriveSignal(now).getLeft() + "\nRight: " + pursuit.getNextDriveSignal(now).getRight() + "\n");
    Robot.kDrivetrain.setFPS(pursuit.getNextDriveSignal(now).getLeft(), pursuit.getNextDriveSignal(now).getRight());
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return pursuit.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
