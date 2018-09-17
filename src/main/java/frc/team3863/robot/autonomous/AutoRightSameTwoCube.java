package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3863.robot.commands.*;
import frc.team3863.robot.util.Odometry;
import jaci.pathfinder.Trajectory;

import static frc.team3863.robot.Robot.paths;

/**
 *
 */
public class AutoRightSameTwoCube extends CommandGroup {

    public AutoRightSameTwoCube() {
        addSequential(new AutoPathFollower(paths.get("rightSameSideScale1")));
        addParallel(new ElevatorSetpoint(4));
        addSequential(new OuttakeCube());
        addSequential(new TurnDegrees(180, 2));
        addParallel(new ElevatorSetpoint(0));
        addSequential(new AutoPathFollower((paths.get("rightSameSideScale2"))));
        addParallel(new IntakeClaw(true));
        addParallel(new EnableIntakeWheels(false, 3));
        addSequential(new IntakeClaw(false));
        addSequential(new TurnDegrees(-180, 2));
        addSequential(new AutoPathFollower(paths.get("rightSideSameScale3")));
        addParallel(new ElevatorSetpoint(4));
        addSequential(new OuttakeCube());
        addSequential(new AutoPathFollower(paths.get("rightSideSameScale4")));
        addParallel(new ElevatorSetpoint(0));
        addSequential(new ZeroLift());
    }

    public Odometry getInitOdometry(){
        Trajectory.Segment s = paths.get("rightSameSideScale1").get(0);
        return new Odometry(s.x, s.y, s.heading);
    }
}
