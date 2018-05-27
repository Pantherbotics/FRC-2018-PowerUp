package frc.team3863.robot.autonomous;

import frc.team3863.robot.commands.DeployIntake;
import frc.team3863.robot.commands.DriveForwardInches;
import frc.team3863.robot.commands.DropCube;
import frc.team3863.robot.commands.ElevatorSetpoint;
import frc.team3863.robot.commands.OuttakeCube;
import frc.team3863.robot.commands.RotateDegrees;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoLeftScale extends CommandGroup {
	int right_invert = 1;
    public AutoLeftScale( boolean start_right) {
    	if (start_right) {
        	right_invert = -1;
        	//System.out.println("Near Auto Right");
        }else {
        	//System.out.println("Near Auto Left");
        }
    	
    	//addSequential(new ElevatorSetpoint(1));
    	//addSequential(new DriveForwardInches(36.0));
    	//addSequential(new RotateDegrees(-90 * right_invert));
    	addSequential(new DriveForwardInches(135.0));
    	addSequential(new ElevatorSetpoint(4));
    	addSequential(new DriveForwardInches(88.0));
    	addSequential(new RotateDegrees(90 * right_invert));
    	//addSequential(new DriveForwardInches(40.5)); //54.5
    	addSequential(new OuttakeCube());
    	//addSequential(new DriveForwardInches(-20.0));
    }
}
