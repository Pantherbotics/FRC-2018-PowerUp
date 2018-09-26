package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3863.robot.util.Odometry;

public class PathedAutonomous extends CommandGroup {

    public Odometry getInitOdometry(){
        return new Odometry(0,0,0);
    }

}
