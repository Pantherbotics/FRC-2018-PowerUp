package frc.team3863.robot.util;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**3863 implementation of https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
/*special thanks to Solomon and Prateek
/*thanks to Jaci for her work on Pathfinder!
 */

public class RamseteFollower {

    static final double b = 2; // greater than zero; increases correction
    static final double zeta = 0.9; // between zero and one; increases dampening
    double k, v, w, w_d, wheelBase, lastTheta;
    int segmentIndex;
    Trajectory path;    //this is the path that we will follow
    Odometry odo;       //this is the robot's x and y position, as well as its heading

    //where v = linear velocity (feet/s) and w = angular velocity (rad/s)
    public RamseteFollower(double wheelBase, Trajectory path) {
        System.out.println("Initializing Ramsete Follower");
        this.wheelBase = wheelBase;
        this.path = path;
        segmentIndex = 0;
    }

    private void calcW_d() {
        if (segmentIndex < path.length()-1) {
            lastTheta = path.get(segmentIndex).heading;
            double nextTheta = path.get(segmentIndex + 1).heading;
            double diffTheta = nextTheta - lastTheta;
            w_d = diffTheta / path.get(segmentIndex).dt;
        } else
            w_d = 0;
    }

    public DriveSignal getNextDriveSignal() {
        double left = 0;
        double right = 0;
        if (isFinished()) {
            return new DriveSignal(left, right);
        }
        System.out.println("Getting segment segmentIndex number: " + segmentIndex + "out of " + (path.length()-1) + " segments" + " is finished: " + isFinished());

        Segment current = path.get(segmentIndex);   //look at segment of path
        calcW_d();   //need to find wanted rate of change of heading

        calcVel(current.x, current.y, current.heading, current.velocity, w_d);
        calcAngleVel(current.x, current.y, current.heading, current.velocity, w_d);

        //System.out.println("Velocity " + v + " Angular Velocity " + w);

        left = (-wheelBase * w) / 2 + v;  //do math to convert angular velocity + linear velocity into left and right wheel speeds (fps)
        right = (+wheelBase * w) / 2 + v;


        System.out.println("Left: " + left + " Right: " + right);
        //left *= -1; //robot was going backwards...? dunno why
        //right *= -1;
        segmentIndex++;
        return new DriveSignal(left, right);
    }

    public void setOdometry(Odometry odometry) {
        odo = odometry;
    }

    private void calcVel(double x_d, double y_d, double theta_d, double v_d, double w_d) {
        calcK(v_d, w_d);
        v = v_d * Math.cos(theta_d - odo.getTheta()) + k * (Math.cos(odo.getTheta()) * (x_d - odo.getX()) + Math.sin(odo.getTheta()) * (y_d - odo.getY()));
    }

    private void calcAngleVel(double x_d, double y_d, double theta_d, double v_d, double w_d) {
        calcK(v_d, w_d);
        System.out.println("Theta" + odo.getTheta());
        double thetaError = theta_d - odo.getTheta();
        double sinThetaErrOverThetaErr;
        if (thetaError < 0.00001)
            sinThetaErrOverThetaErr = 1; //this is the limit as sin(x)/x approaches zero
        else
            sinThetaErrOverThetaErr = Math.sin(theta_d - odo.getTheta()) / (thetaError);
        double calcW = w_d + b * v_d * (sinThetaErrOverThetaErr) * (Math.cos(odo.getTheta()) * (y_d - odo.getY()) - Math.sin(odo.getTheta()) * (x_d - odo.getX())) + k * (thetaError); //from eq. 5.12
        w = calcW % (Math.PI); // bind it! [-2pi, 2pi]
    }

    private void calcK(double v_d, double w_d) {
        k = 2 * zeta * Math.sqrt(Math.pow(w_d, 2) + b * Math.pow(v_d, 2)); //from eq. 5.12
    }

    public Odometry getInitOdometry() {
        return new Odometry(path.get(0).x,path.get(0).y, path.get(0).heading);
    }

    public boolean isFinished() {
        return segmentIndex == path.length();
    }


}