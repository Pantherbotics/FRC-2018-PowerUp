package org.usfirst.frc.team3863.robot.teleop;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TeleopSinglePartnerController extends Command {

    public TeleopSinglePartnerController() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.kDrivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Partner Controller Drive enabled");
    	Robot.m_oi.initSinglePartnerController();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double twist = Robot.m_oi.partnerController.getZ();
    	double y = Robot.m_oi.partnerController.getY();
    	if (Math.abs(twist) <= Constants.CONTROLLER_DEADBAND) { twist = 0;}
    	if (Math.abs(y) <= Constants.CONTROLLER_DEADBAND) { y = 0;}
    	
    	double left = y - twist;
    	double right = y + twist;
    	//Robot.kDrivetrain.setVelocityTargets(left, right);
    	Robot.kDrivetrain.setDrivePower(left, right);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
