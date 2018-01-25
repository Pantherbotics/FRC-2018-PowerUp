package org.usfirst.frc.team3863.robot.commands;

import org.usfirst.frc.team3863.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *Autonomous for placing a power cube in the Switch, when our switch side is the same as our starting side. 
 * This auton should be written for the Left side of the field.
 * if start_right is True, then left-right values should be inverted (using right_invert), as we are starting on the right side of the field
 */
public class SwitchNearLeftAuto extends Command {
	int right_invert = 1;
    public SwitchNearLeftAuto(boolean start_right) {
        requires(Robot.kDrivetrain);
        if (start_right) {
        	right_invert = -1;
        }
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (right_invert == 1) {
    		System.out.println("Autosomis Mode, Auton: Score in CLOSEST switch side, starting on LEFT of field");
    		SmartDashboard.putString("Autosomis Mode", "Auton: Score in CLOSEST switch side, starting on LEFT of field");
    		
    	} else if (right_invert == -1) {
    		System.out.println("Autosomis Mode, Auton: Score in CLOSEST switch side, starting on RIGHT of field");
    		SmartDashboard.putString("Autosomis Mode", "Auton: Score in CLOSEST switch side, starting on RIGHT of field");
    		
    		


    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
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
