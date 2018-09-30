package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3863.robot.commands.DropCube;
import frc.team3863.robot.commands.ElevatorSetpoint;
import frc.team3863.robot.commands.OuttakeCube;
import frc.team3863.robot.util.Odometry;
import jaci.pathfinder.Trajectory;

import static frc.team3863.robot.Robot.paths;

public class AutoCenterLeftOneCube extends PathedAutonomous {

    public AutoCenterLeftOneCube(){
        addParallel(new ElevatorSetpoint(3));
        addSequential(new AutoPathFollower(paths.get("centerLeftSwitch")));
        addSequential(new OuttakeCube());
    }

    public Odometry getInitOdometry(){
        Trajectory.Segment s = paths.get("centerLeftSwitch").get(0);
        return new Odometry(s.x, s.y, s.heading);
    }
}
