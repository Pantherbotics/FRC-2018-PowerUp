package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3863.robot.commands.DriveForwardInches;

/**
 *
 */
public class AutoBaseline extends CommandGroup {

    public AutoBaseline() {
        addSequential(new DriveForwardInches(120.0));
    }
}
