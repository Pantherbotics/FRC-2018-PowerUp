package org.usfirst.frc.team3863.robot.teleop;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.Robot;
import org.usfirst.frc.team3863.robot.commands.ElevatorSetpoint;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TeleopSinglePartnerController extends Command {
    boolean usepid;
    Integer lastPOV;
    public TeleopSinglePartnerController(boolean usePIDDrive) {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.kDrivetrain);
    	usepid = usePIDDrive;
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
    	int pov = Robot.m_oi.partnerController.getPOV();
    	if (Math.abs(twist) <= Constants.CONTROLLER_DEADBAND) { twist = 0;}
    	if (Math.abs(y) <= Constants.CONTROLLER_DEADBAND) { y = 0;}
    	
    	if (lastPOV == null || pov != lastPOV) {
    		Command povCommand = null;
    		switch (pov){
	    		case 0:
	    			povCommand = new ElevatorSetpoint(5); //Top
	    			break;
	    		case 90:
	    			povCommand = new ElevatorSetpoint(1); //Raised
	    			break;
	    		case 180:
	    			povCommand = new ElevatorSetpoint(0); //Bottom
	    			break;
	    		case 270:
	    			povCommand = new ElevatorSetpoint(3); //Switch
	    			break;
    		}
    		if (povCommand != null) {
    			povCommand.start();
    		}
    		lastPOV = pov;
    	}
    	
    	double elevDampen = 1.0 - Robot.kElevator.getHeightPercent();
    	if (elevDampen < 0.6) {
    		elevDampen = 0.6;
    	}
    	
    	double left = (y - twist) * elevDampen;
    	double right = (y + twist) * elevDampen;
    	if (usepid) {
    		Robot.kDrivetrain.setVelocityTargets(left, right);
    	}else {
    		Robot.kDrivetrain.setDrivePower(left, right);
    	}
    	
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
