package frc.team3863.robot.autonomous;

import frc.team3863.robot.commands.DeployIntake;
import frc.team3863.robot.commands.DriveForwardInches;
import frc.team3863.robot.commands.ElevatorSetpoint;
import frc.team3863.robot.commands.OuttakeCube;
import frc.team3863.robot.commands.RotateDegrees;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftSwitchFar extends CommandGroup {
	int right_invert = 1;
    public AutoLeftSwitchFar( boolean start_right) {
    	if (start_right) {
        	right_invert = -1;
        	//System.out.println("Far Auto Right");
        }else {
        	//System.out.println("Far Auto Left");
        }
    	addSequential(new ElevatorSetpoint(1));
    	addSequential(new DriveForwardInches(230.0));
    	addSequential(new RotateDegrees(90 * right_invert));
    	addSequential(new DriveForwardInches(128.0)); //real field: 200
    	addSequential(new ElevatorSetpoint(3));
    	addSequential(new RotateDegrees(90 * right_invert));
    	addSequential(new OuttakeCube());
    	addSequential(new DriveForwardInches(-20.0));
    }
}
