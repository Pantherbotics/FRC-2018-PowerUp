package frc.team3863.robot.autonomous;

import frc.team3863.robot.commands.DriveForwardInches;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoBaseline extends CommandGroup {

    public AutoBaseline() {
        addSequential(new DriveForwardInches(120.0));
    }
}
