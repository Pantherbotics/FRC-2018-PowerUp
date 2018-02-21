package org.usfirst.frc.team3863.robot.autonomous;

import org.usfirst.frc.team3863.robot.commands.DriveForwardInches;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoBaseline extends CommandGroup {

    public AutoBaseline() {
        addSequential(new DriveForwardInches(95.0));
    }
}
