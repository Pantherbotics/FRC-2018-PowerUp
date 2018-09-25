package frc.team3863.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3863.robot.Constants;
import frc.team3863.robot.RobotMap;
import frc.team3863.robot.util.Odometry;
import frc.team3863.robot.util.Units;

/**
 * Controls the four CANTalons dedicated to the Drivetrain
 */
public class Drivetrain extends Subsystem {

    public int timeout_ms = 0;
    public boolean transmission_in_low = true;
    // ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    AHRS ahrs_gyro = new AHRS(I2C.Port.kOnboard);
    TalonSRX talonLeftA = new TalonSRX(RobotMap.TALON_DRIVE_LEFTA_ID);
    TalonSRX talonLeftB = new TalonSRX(RobotMap.TALON_DRIVE_LEFTB_ID);
    TalonSRX talonRightA = new TalonSRX(RobotMap.TALON_DRIVE_RIGHTA_ID);
    TalonSRX talonRightB = new TalonSRX(RobotMap.TALON_DRIVE_RIGHTB_ID);
    DoubleSolenoid transmissiom_solenoid = new DoubleSolenoid(RobotMap.PCM_TRANSMISSION_LOW,
            RobotMap.PCM_TRANSMISSION_HIGH);
    int high_pid_id = 0;
    int low_pid_id = 1;

    //variables for Odometry
    private double lastPos, currentPos, dPos;
    volatile double x, y, theta;


    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void init() {                                                                                //note to self: i need to invert drivetrain direction, but not sure if need to also invert sensor phase?
        talonLeftA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeout_ms);
        talonLeftA.setSensorPhase(false);

        talonRightA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeout_ms);
        talonRightA.setSensorPhase(false);

        talonLeftA.setInverted(false);
        talonLeftB.setInverted(false);

        talonRightA.setInverted(true);
        talonRightB.setInverted(true);

        talonRightA.setNeutralMode(NeutralMode.Coast);
        talonRightB.setNeutralMode(NeutralMode.Coast);
        talonLeftA.setNeutralMode(NeutralMode.Coast);
        talonLeftB.setNeutralMode(NeutralMode.Coast);

        talonLeftB.follow(talonLeftA);
        talonRightB.follow(talonRightA);


        talonLeftA.configClosedloopRamp(Constants.DRIVE_RAMP_SECONDS, timeout_ms);
        talonLeftB.configClosedloopRamp(Constants.DRIVE_RAMP_SECONDS, timeout_ms);
        talonRightA.configClosedloopRamp(Constants.DRIVE_RAMP_SECONDS, timeout_ms);
        talonRightB.configClosedloopRamp(Constants.DRIVE_RAMP_SECONDS, timeout_ms);

        initPID();

        zeroGyro();

        setTransmissionLow();
        talonLeftA.configPeakCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_PEAK, 0);
        talonLeftA.configPeakCurrentDuration(Constants.DRIVE_CURRENT_LIMIT_PEAK_TIME, 0);
        talonLeftA.configContinuousCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_SUSTAINED, 0);

        talonRightA.configPeakCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_PEAK, 0);
        talonRightA.configPeakCurrentDuration(Constants.DRIVE_CURRENT_LIMIT_PEAK_TIME, 0);
        talonRightA.configContinuousCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_SUSTAINED, 0);

        talonLeftA.enableCurrentLimit(true);
        talonRightA.enableCurrentLimit(true);

        talonLeftA.setSelectedSensorPosition(0, 0, 0);
        talonRightA.setSelectedSensorPosition(0, 0, 0);

        talonLeftA.configMotionAcceleration(Constants.MOTION_MAGIC_ACCEL, 0);
        talonLeftA.configMotionCruiseVelocity(Constants.MOTION_MAGIC_VEL, 0);

        talonRightA.configMotionAcceleration(Constants.MOTION_MAGIC_ACCEL, 0);
        talonRightA.configMotionCruiseVelocity(Constants.MOTION_MAGIC_VEL, 0);

        talonLeftA.enableVoltageCompensation(true);
        talonRightA.enableVoltageCompensation(true);
        x = 0;
        y = 0;
        theta = 0;

        lastPos = (talonLeftA.getSelectedSensorPosition(0) + talonRightA.getSelectedSensorPosition(0))/2;

        Notifier odoThread = new Notifier(() ->{
            currentPos = (talonLeftA.getSelectedSensorPosition(0) + talonRightA.getSelectedSensorPosition(0))/2;
            dPos = Units.TalonNativeToFeet(currentPos - lastPos);
            theta = Math.toRadians(boundHalfDegrees(-ahrs_gyro.getAngle()));
            x +=  Math.cos(theta) * dPos;
            y +=  Math.sin(theta) * dPos;
            lastPos = currentPos;
        });

        odoThread.startPeriodic(0.01);
    }

    private void initPID() {
        talonLeftA.configAllowableClosedloopError(0, high_pid_id, timeout_ms);
        talonLeftA.config_kF(high_pid_id, Constants.HIGH_DRIVE_PID_F, timeout_ms);
        talonLeftA.config_kP(high_pid_id, Constants.HIGH_DRIVE_PID_P, timeout_ms);
        talonLeftA.config_kI(high_pid_id, Constants.HIGH_DRIVE_PID_I, timeout_ms);
        talonLeftA.config_kD(high_pid_id, Constants.HIGH_DRIVE_PID_D, timeout_ms);


        talonRightA.configAllowableClosedloopError(0, high_pid_id, timeout_ms);
        talonRightA.config_kF(high_pid_id, Constants.HIGH_DRIVE_PID_F, timeout_ms);
        talonRightA.config_kP(high_pid_id, Constants.HIGH_DRIVE_PID_P, timeout_ms);
        talonRightA.config_kI(high_pid_id, Constants.HIGH_DRIVE_PID_I, timeout_ms);
        talonRightA.config_kD(high_pid_id, Constants.HIGH_DRIVE_PID_D, timeout_ms);

        talonLeftA.configAllowableClosedloopError(0, low_pid_id, timeout_ms);
        talonLeftA.config_kF(low_pid_id, Constants.LOW_DRIVE_PID_F, timeout_ms);
        talonLeftA.config_kP(low_pid_id, Constants.LOW_DRIVE_PID_P, timeout_ms);
        talonLeftA.config_kI(low_pid_id, Constants.LOW_DRIVE_PID_I, timeout_ms);
        talonLeftA.config_kD(low_pid_id, Constants.LOW_DRIVE_PID_D, timeout_ms);

        talonRightA.configAllowableClosedloopError(0, low_pid_id, timeout_ms);
        talonRightA.config_kF(low_pid_id, Constants.LOW_DRIVE_PID_F, timeout_ms);
        talonRightA.config_kP(low_pid_id, Constants.LOW_DRIVE_PID_P, timeout_ms);
        talonRightA.config_kI(low_pid_id, Constants.LOW_DRIVE_PID_I, timeout_ms);
        talonRightA.config_kD(low_pid_id, Constants.LOW_DRIVE_PID_D, timeout_ms);

    }

    private void setPIDProfile(int id) {
        talonLeftA.selectProfileSlot(id, 0);
        talonRightA.selectProfileSlot(id, 0);
    }

    public double[] getEncoderVelocities() {
        double l = talonLeftA.getSelectedSensorVelocity(timeout_ms);
        double r = talonRightA.getSelectedSensorVelocity(timeout_ms);
        return new double[]{l, r};
    }

    public double[] getEncoderPositions() {
        double l = talonLeftA.getSelectedSensorPosition(timeout_ms);
        double r = talonRightA.getSelectedSensorPosition(timeout_ms);
        return new double[]{l, r};
    }

    public double[] getDriveVoltages(){
        double l = talonLeftA.getMotorOutputVoltage();
        double r = talonRightA.getMotorOutputVoltage();
        return new double[]{l,r};
    }

    public void setDrivePower(double left, double right) {
        // Set the left and right motor power
        talonLeftA.set(ControlMode.PercentOutput, left);
        talonRightA.set(ControlMode.PercentOutput, right);
    }

    public void setFPS(double left, double right) {
        /*
        double actualLeft = Units.TalonNativeToFPS(talonLeftA.getSelectedSensorVelocity(0));
        double actualRight = Units.TalonNativeToFPS(talonRightA.getSelectedSensorVelocity(0));
        if ( actualLeft > LOW_GEAR_TOP_SPEED-2 || actualRight > LOW_GEAR_TOP_SPEED-2) {
            setTransmissionHigh();
        }

        if (actualLeft < LOW_GEAR_TOP_SPEED - 4 && actualRight < LOW_GEAR_TOP_SPEED - 4) {
            setTransmissionLow();
        }*/
        setTransmissionHigh();

        //System.out.println("wanted " + left + " " + right);
        //System.out.println("real " + Units.TalonNativeToFPS(talonLeftA.getSelectedSensorVelocity(0)) + " " + Units.TalonNativeToFPS(talonRightA.getSelectedSensorVelocity(0)));
        talonLeftA.set(ControlMode.Velocity, Units.FPSToTalonNative(left), DemandType.ArbitraryFeedForward, Constants.HIGH_DRIVE_PID_F_INTERCEPT);
        talonRightA.set(ControlMode.Velocity, Units.FPSToTalonNative(right), DemandType.ArbitraryFeedForward, Constants.HIGH_DRIVE_PID_F_INTERCEPT);
    }

    public void setPositionTargetIncrements(double leftOffset, double rightOffset) {
        double lTarget = talonLeftA.getSelectedSensorPosition(0) + leftOffset;
        double rTarget = talonRightA.getSelectedSensorPosition(0) + rightOffset;
        talonLeftA.set(ControlMode.MotionMagic, lTarget);
        talonRightA.set(ControlMode.MotionMagic, rTarget);
    }

    public void setTransmissionLow() {
        //System.out.println("Transmission in Low Gear");
        transmission_in_low = true;
        setPIDProfile(low_pid_id);
        transmissiom_solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setTransmissionHigh() {
        //System.out.println("Transmission in High Gear");
        transmission_in_low = false;
        setPIDProfile(high_pid_id);
        transmissiom_solenoid.set(DoubleSolenoid.Value.kReverse);

    }

    public double pidErrorAverage() {
        return (double) ((talonLeftA.getClosedLoopError(timeout_ms) + talonRightA.getClosedLoopError(timeout_ms)) / 2);

    }

    public void zeroEncoderPositions() {
        talonLeftA.setSelectedSensorPosition(0, high_pid_id, timeout_ms);
        talonRightA.setSelectedSensorPosition(0, high_pid_id, timeout_ms);
        talonLeftA.set(ControlMode.Position, 0);
        talonRightA.set(ControlMode.Position, 0);

    }

    public void zeroGyro() {
        System.out.print("Zeroing Gyro...");
        ahrs_gyro.reset();
        System.out.println("...Zeroing Complete");
    }

    public Odometry getOdometry() {
        return new Odometry(x, y, theta);
    }

    public void setOdometry(Odometry odo){
        this.x = odo.getX();
        this.y = odo.getY();
        this.theta = odo.getTheta();
    }

    public String toString(){
        return "X: " + x + " Y: " + y + "Theta: "+ theta;
    }

    private double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }
}
