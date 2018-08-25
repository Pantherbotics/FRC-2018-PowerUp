package frc.team3863.robot.autonomous;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

import java.util.ArrayList;

public class Paths {

    public static Waypoint[] CenterSwitch = new Waypoint[]{
            new Waypoint(0, 0, 0), new Waypoint(4, 0, 0), new Waypoint(6, -2, -Math.PI / 2)
    };
    static ArrayList<Trajectory> allTraj;
    static Trajectory ftraj;
    Trajectory.Config configuration;
    double width;
    TankModifier mod;

    public Paths(Trajectory path, Trajectory.Config config, double wheelbase_width) {
        configuration = config;
        width = wheelbase_width;
        mod = new TankModifier(path);
        mod.modify(width);
    }

    public static void addTraj(Trajectory traj) {
        ftraj = traj;
    }

    public static Trajectory getLastTraj() {
        return ftraj;
    }


}
