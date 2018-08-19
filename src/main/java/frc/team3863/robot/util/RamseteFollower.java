package frc.team3863.robot.util;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogGyro;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import frc.team3863.robot.util.DriveSignal;
import com.kauailabs.navx.frc.AHRS;

//3863 implementation of https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
//special thanks to Solomon and Prateek
//thanks to Jaci for her work on Pathfinder!

public class RamseteFollower{

    TalonSRX left, right;
    double k_1, k_2, k_3, v, w, v_d, w_d, theta_d, x_d, y_d;
    double wheelBase;
    AHRS gyro;
    Trajectory path;

    volatile double x, y, theta;

    static final double b = 2; // greater than zero
    static final double zeta = 0.7; // between zero and one
    static final double wheelDiameter = 6;
    static final double HIGH_GEAR_MAX_SPEED = 20;

    int index, numSegments;
    //where v = linear velocity (feet/s) and w = angular velocity (rad/s)
    public RamseteFollower(TalonSRX left, TalonSRX right, double wheelBase, Trajectory path, AHRS gyro){
    	System.out.println("initializing follower");
        this.left = left;
        this.right = right;
        this.gyro = gyro;
        this.wheelBase = wheelBase;
        this.path = path;
        k_2 = b;
        index = 0;
        numSegments = path.length()-1;
    }

    public void setOdometry(double x, double y) {
    	this.x = x;
    	this.y = y;
    }
    public DriveSignal getNextWheelCommand(){
    	double left = 0;
        double right = 0;
        if(index == numSegments-1){
            return new DriveSignal(left, right);
        }
        System.out.println("Getting segment index number: " + index + "out of " + path.length() + " segments");
        
        Segment current = path.get(index);
        index++;
        calcVel(current.x, current.y, current.heading, current.velocity, current.heading/current.dt);
        calcAngleVel(current.x, current.y, current.heading, current.velocity, current.heading/current.dt);

        System.out.println(current.heading/current.dt);
        System.out.println("Velocity " + v + " Angular Velocity " + w);

        left = (-wheelBase*w)/2 + v;
        right = (+wheelBase*w)/2 + v;

        left /= HIGH_GEAR_MAX_SPEED;
        right /=HIGH_GEAR_MAX_SPEED;
        
        left *= -1;
        right *= -1;
        return new DriveSignal(left, right);
    }

    public void setOdometry(double[] odometry){
        x = odometry[0];
        y = odometry[1];
        theta = odometry[2];
    }
    public void calcVel(double x_d, double y_d, double theta_d, double v_d, double w_d){
        calcK(v_d, w_d);
        double calcV = v_d * Math.cos(theta_d - Math.toRadians(gyro.getAngle())) + k_1 * (Math.cos(theta)*(x_d - x) + Math.sin(theta)*(y_d-y));
        v = calcV;
    }

    public void calcAngleVel(double x_d, double y_d, double theta_d, double v_d, double w_d){
        calcK(v_d, w_d);
        theta = Math.toRadians(gyro.getAngle()) % Math.PI/2;
        System.out.println("Theta " + theta);
        double thetaError = theta_d-theta;
        if(thetaError < 0.001)
        	theta = .0001;
        double calcW = w_d + k_2 * v_d * (Math.sin(theta_d-theta) / (thetaError)) * (Math.cos(y_d - y) - Math.sin(theta)*(x_d - x)) + k_3 * (thetaError);
        w = calcW;
    }

    public void calcK(double v_d, double w_d){
        k_1 = 2 * zeta * Math.sqrt(Math.pow(w_d,2)+ k_2*Math.pow(v_d, 2));
        k_3 = k_1;
    }

    public boolean isFinished(){
        return index == numSegments+1;
    }


}