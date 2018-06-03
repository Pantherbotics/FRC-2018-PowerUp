package frc.team3863.robot.autonomous;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class Paths {

    Trajectory.Config configuration;
    double width;
    TankModifier mod;
    public Paths(Trajectory path, Trajectory.Config config, double wheelbase_width){
        configuration = config;
        width = wheelbase_width;
        mod = new TankModifier(path);
        mod.modify(width);
    }

    public static Waypoint[] CenterSwitch = new Waypoint[]{
      new Waypoint(0, 0, 0), new Waypoint(3, 0, 0), new Waypoint (5, -1, -90), new Waypoint (5, -5, -90)
    };

    public Trajectory getLeft(){
        return mod.getLeftTrajectory();
    }

    public Trajectory getRight(){
        return mod.getRightTrajectory();
    }

}
