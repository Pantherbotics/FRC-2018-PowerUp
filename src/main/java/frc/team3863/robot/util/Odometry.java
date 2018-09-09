package frc.team3863.robot.util;

public class Odometry {
    private double x, y, theta;

    public Odometry(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }


    public double getTheta() {
        return theta % (Math.PI * 2.0);
    }

}
