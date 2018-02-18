package org.usfirst.frc.team3863.robot.commands;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoIntake extends Command {
	int state = 0;
	int counter = 0;
	boolean isComplete = false;
    public AutoIntake() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	state = 0;
    	isComplete = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	switch (state) {
	    	case 0:
	    		if (Robot.kIntake.isCubeInIntake()) {
					state = 6;
				}else {
					Robot.kElevator.goToPreset(0);
					Robot.kIntake.openClaw();
					state = 1;
				}
				break;
				
			case 1:
				if (Robot.kIntake.isCubeInfrontOfIntake()) {
					state = 2;
				}
				break;
	    	
	    	case 2:
	    		Robot.kIntake.closeClaw();
	    		Robot.kIntake.setIntakeWheelPower(-Constants.INTAKE_MOTOR_POWER);
	    		if (Robot.kIntake.isCubeInIntake()) {
					state = 3;
				}
	    		break;
	    		
	    	case 3:
	    		if (Robot.kIntake.testMotorCurrentThreshold(9.8)) {
					state = 4;
				}
	    		
	    	case 4:
	    		if (counter > 40) {
	    			state = 5;
	    			counter = 0;
	    		}else {
	    			counter += 1;
	    		}
	    		
	    	case 5:
	    		Robot.kIntake.setIntakeWheelPower(0);
	    		Robot.kElevator.goToPreset(1);
	    		state = 6;
	    		
	    	case 6:
	    		isComplete  = true;
    	}
	    		
    		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isComplete;
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	state = 0;
    	isComplete = true;
    }
}
