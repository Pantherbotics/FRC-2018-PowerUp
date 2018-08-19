package frc.team3863.robot.autonomous;

import java.util.ArrayList;


import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class Paths {

    Trajectory.Config configuration;
    double width;
    TankModifier mod;
    static ArrayList<Trajectory> allTraj;
    static Trajectory ftraj;
    public Paths(Trajectory path, Trajectory.Config config, double wheelbase_width){
        configuration = config;
        width = wheelbase_width;
        mod = new TankModifier(path);
        mod.modify(width);
    }

    public static Waypoint[] CenterSwitch = new Waypoint[]{
      new Waypoint(0, 0, 0), new Waypoint(4, 0, 0), new Waypoint(6, -2, -Math.PI/2)
    };
    
    public static void addTraj(Trajectory traj) {
    	ftraj = traj;
    }
    
    public static Trajectory getLastTraj() {
    	return ftraj;
    }
    

}
