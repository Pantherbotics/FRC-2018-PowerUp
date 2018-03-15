package org.usfirst.frc.team3863.robot.autonomous;

import org.usfirst.frc.team3863.robot.commands.DriveForwardInches;
import org.usfirst.frc.team3863.robot.commands.RotateDegrees;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftBaselineCenter extends CommandGroup {

	int right_invert = 1;
    public AutoLeftBaselineCenter(boolean turn_to_right) {
    	if (turn_to_right) {
        	right_invert = -1;
        }
    	addSequential(new RotateDegrees(-45 * right_invert));
    	addSequential(new DriveForwardInches(198));
    }
}
