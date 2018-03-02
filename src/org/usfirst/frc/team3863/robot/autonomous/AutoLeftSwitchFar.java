package org.usfirst.frc.team3863.robot.autonomous;

import org.usfirst.frc.team3863.robot.commands.DeployIntake;
import org.usfirst.frc.team3863.robot.commands.DriveForwardInches;
import org.usfirst.frc.team3863.robot.commands.OuttakeCube;
import org.usfirst.frc.team3863.robot.commands.RotateDegrees;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftSwitchFar extends CommandGroup {
	int right_invert = 1;
    public AutoLeftSwitchFar( boolean start_right) {
    	if (start_right) {
        	right_invert = -1;
        	System.out.println("Far Auto Right");
        }else {
        	System.out.println("Far Auto Left");
        }
    	//addSequential(new DeployIntake());
    	//addSequential(new DriveForwardInches(95.0));
    	//addSequential(new RotateDegrees(90 * right_invert));
    	//addSequential(new OuttakeCube());
    }
}
