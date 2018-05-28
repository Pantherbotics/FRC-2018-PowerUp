package frc.team3863.robot.autonomous;

import frc.team3863.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftSwitchCenter extends CommandGroup {
	int right_invert = 1;
    public AutoLeftSwitchCenter( boolean start_right) {
    	if (start_right) {
        	right_invert = -1;
        	//System.out.println("Near Auto Right");
        }else {
        	//System.out.println("Near Auto Left");
        }
    	
    	//addSequential(new ElevatorSetpoint(1));
    	addSequential(new DriveForwardInches(36.0));
    	addSequential(new RotatePIDDegrees(-90 * right_invert, 5));
    	if (start_right) {
    		addSequential(new DriveForwardInches(30.0)); //54
    	}else {
    		addSequential(new DriveForwardInches(45.0)); //54
    	}
    	
    	addSequential(new RotatePIDDegrees(90 * right_invert, 5));
    	addSequential(new ElevatorSetpoint(3));
    	addSequential(new DriveForwardInches(40)); //54.5
    	addSequential(new DropCube());
    	addSequential(new DriveForwardInches(-20.0));
    }
}
