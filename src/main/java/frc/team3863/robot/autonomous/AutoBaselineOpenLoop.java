package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3863.robot.commands.DriveForwardOpenLoop;

/**
 *
 */
public class AutoBaselineOpenLoop extends CommandGroup {

    public AutoBaselineOpenLoop() {
        addSequential(new DriveForwardOpenLoop(75));
    }
}
