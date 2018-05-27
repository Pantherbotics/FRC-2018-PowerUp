package frc.team3863.robot.autonomous;

import frc.team3863.robot.commands.DriveForwardInches;
import frc.team3863.robot.commands.DriveForwardOpenLoop;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoBaselineOpenLoop extends CommandGroup {

    public AutoBaselineOpenLoop() {
        addSequential(new DriveForwardOpenLoop(75));
    }
}
