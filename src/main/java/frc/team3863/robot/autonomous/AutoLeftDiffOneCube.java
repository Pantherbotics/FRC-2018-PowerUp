package frc.team3863.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team3863.robot.Robot;
import frc.team3863.robot.commands.ElevatorSetpoint;
import frc.team3863.robot.commands.IntakeClaw;
import frc.team3863.robot.commands.OuttakeCube;
import frc.team3863.robot.util.Odometry;
import jaci.pathfinder.Trajectory;


public class AutoLeftDiffOneCube extends PathedAutonomous {

    public AutoLeftDiffOneCube() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the arm.
        addParallel(new ElevatorSetpoint(5, 2));
        addSequential(new AutoPathFollower(Robot.paths.get("leftDiffSideScale1")));
        addSequential(new OuttakeCube());
    }

    public Odometry getInitOdometry(){
        Trajectory.Segment s = Robot.paths.get("leftDiffSideScale1").get(0);
        return new Odometry(s.x, s.y, s.heading);
    }
}
