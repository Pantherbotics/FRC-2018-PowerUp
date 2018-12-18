package frc.team3863.robot.util;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/*  3863 implementation of PurePursuit
    https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
    
*/

public class PurePursuit{ //This is probably the worst thing I [Matthew] have ever written. -MS
    
    private int indexI;
    private int indexF;
    private Trajectory path;
    private double wheelBase;
    private double lookahead;
    private Odometry odom;
    double left = 0;
    double right = 0;
    double dt = 0;

    public PurePursuit(double wheelBase, Trajectory path){
        this.wheelBase = wheelBase;
        this.path = path;
        indexI = 0;
        indexF = 0;
    }

    public DriveSignal getNextDriveSignal(double elapsed){
        
        if(isFinished()){
            return new DriveSignal(left, right);
        }

        indexI = findSetpoint(path.get(0).dt, elapsed, true);
        indexF = findSetpoint(path.get(0).dt, elapsed, false);

        dt = path.get(indexF).heading - path.get(indexI).heading;

        Segment current = path.get(indexF);

        //double v = calcVel(current.x, current.y, current.heading, current.velocity, calc_wD(dT, indexI, indexF), dT);
        //double w = calcAngleVel(current.x, current.y, current.heading, current.velocity, calc_wD(dT, indexI, indexF), dT);

        //left = (wheelBase * w) / 2 + v;
        //right = (wheelBase * w) / 2 + v;
 
        return calcVel(current.x, current.y, dt);
    }

    //Finds the segment index of either the lookahead point or the robot. Returns robot point when "robot" is true
    private int findSetpoint(double dT, double elapsed, boolean robot){
        if(robot){
            return Math.toIntExact(Math.round(((path.length() * dT) + elapsed) / dT));
        } else{
            return Math.toIntExact(Math.round(((path.length() * dT) - (elapsed + lookahead)) / dT));
        }    
    }

    //Gives distance of left and right arcs, Set true if left side
    private double calcCircle(double dtheta, double xD, double yD, boolean left){ 
        if(left){ //left
            return Math.abs(dtheta)*(calcR(xD, yD) - (dtheta/Math.abs(dtheta))*wheelBase); //greater when negative
        } else{ //right
            return Math.abs(dtheta)*(calcR(xD, yD) + (dtheta/Math.abs(dtheta))*wheelBase); //greater when positive
        }
    }

    //Return velocities
    private DriveSignal calcVel(double xD, double yD, double dt){
        left = calcCircle(dt, xD, yD, true)/dt;
        right = calcCircle(dt, xD, yD, false)/dt;

        return new DriveSignal(left, right);
    }

    private double calcR(double xD, double yD){ // returns m^-1 (Kappa)
        double dx = xD - odom.getX(); // delta x
        double l = Math.sqrt(Math.pow(xD, 2) + Math.pow(yD, 2));

        return Math.pow(l, 2)/(2 * dx); // curvature
    }

    public void setOdometry(Odometry odometry){
        odom = odometry;
    }

    public Odometry getInitOdometry() {
        return new Odometry(path.get(0).x,path.get(0).y, path.get(0).heading);
    }

    public boolean isFinished(){
        return indexI == path.length();
    }
}