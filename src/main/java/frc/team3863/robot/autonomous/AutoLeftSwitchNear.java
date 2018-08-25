package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3863.robot.commands.DriveForwardInches;
import frc.team3863.robot.commands.ElevatorSetpoint;
import frc.team3863.robot.commands.OuttakeCube;
import frc.team3863.robot.commands.RotateDegrees;

/**
 *
 */
public class AutoLeftSwitchNear extends CommandGroup {
    int right_invert = 1;

    public AutoLeftSwitchNear(boolean start_right) {
        if (start_right) {
            right_invert = -1;
            //System.out.println("Near Auto Right");
        } else {
            //System.out.println("Near Auto Left");
        }

        addSequential(new DriveForwardInches(135.0));
        addSequential(new ElevatorSetpoint(3));
        addSequential(new RotateDegrees(90 * right_invert));
        addSequential(new OuttakeCube());
        addSequential(new DriveForwardInches(-20.0));
    }
}
