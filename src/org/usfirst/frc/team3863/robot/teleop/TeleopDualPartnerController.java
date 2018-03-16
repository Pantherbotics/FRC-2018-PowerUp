package org.usfirst.frc.team3863.robot.teleop;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.Robot;
import org.usfirst.frc.team3863.robot.commands.ElevatorSetpoint;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TeleopDualPartnerController extends Command {
    boolean usepid;
    Integer lastPOV;
    public TeleopDualPartnerController(boolean usePIDDrive) {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.kDrivetrain);
    	usepid = usePIDDrive;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("(COMP) Dual Partner Controller Drive enabled");
    	Robot.m_oi.initDualPartnerController();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double twist = Robot.m_oi.partnerController.getZ();
    	double y = Robot.m_oi.partnerController.getY();
    	double partnerY = Robot.m_oi.auxPartnerController.getY();
    	int auxPov = Robot.m_oi.auxPartnerController.getPOV();
    	if (Math.abs(twist) <= Constants.CONTROLLER_DEADBAND) { twist = 0;}
    	if (Math.abs(y) <= Constants.CONTROLLER_DEADBAND) { y = 0;}
    	if (Math.abs(partnerY) <= Constants.CONTROLLER_DEADBAND) { partnerY = 0;}
    	
    	if (lastPOV == null || auxPov != lastPOV) {
    		Command povCommand = null;
    		switch (auxPov){
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
    		lastPOV = auxPov;
    	}
    	
    	if (Math.abs(partnerY) > 0.05) {
    		int pos_increment = (int) Math.round(Constants.ELEVATOR_DRIVE_INCREMENT * -partnerY);
    		Robot.kElevator.setTargetPosition(Robot.kElevator.target + pos_increment);
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
    	    	
    	if (Robot.m_oi.auxPartnerStart.get() && Robot.m_oi.auxPartnerBack.get()) {
    		if (!Robot.kRamps.is_left_ramp_deployed) {
    			Robot.kRamps.deployLeftRamp();
    		}
    		if (!Robot.kRamps.is_right_ramp_deployed) {
    			Robot.kRamps.deployRightRamp();
    		}
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
