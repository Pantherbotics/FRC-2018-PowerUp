package frc.team3863.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.team3863.robot.Constants;
import frc.team3863.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Controls the four CANTalons dedicated to the Drivetrain
 */
public class Drivetrain extends Subsystem {

	// ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	AHRS ahrs_gyro = new AHRS(I2C.Port.kOnboard);

	WPI_TalonSRX talonLeftA = new WPI_TalonSRX(RobotMap.TALON_DRIVE_LEFTA_ID);
	WPI_TalonSRX talonLeftB = new WPI_TalonSRX(RobotMap.TALON_DRIVE_LEFTB_ID);
	WPI_TalonSRX talonRightA = new WPI_TalonSRX(RobotMap.TALON_DRIVE_RIGHTA_ID);
	WPI_TalonSRX talonRightB = new WPI_TalonSRX(RobotMap.TALON_DRIVE_RIGHTB_ID);

	DoubleSolenoid transmissiom_solenoid = new DoubleSolenoid(RobotMap.PCM_TRANSMISSION_LOW,
			RobotMap.PCM_TRANSMISSION_HIGH);

	int high_pid_id = 0;
	int low_pid_id = 1;
	public int timeout_ms = 0;

	volatile double x, y, theta;

	double HIGH_GEAR_TOP_SPEED = 20;
	double LOW_GEAR_TOP_SPEED = 7;

	public boolean transmission_in_low = true;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void init() {
		talonLeftA.setInverted(true);
		talonLeftB.setInverted(true);

		talonLeftA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, high_pid_id, timeout_ms);
		talonLeftA.setSensorPhase(false);

		talonRightA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, high_pid_id, timeout_ms);
		talonRightA.setSensorPhase(false);

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

		initPID(high_pid_id);

		zero_gyro();

		setTransmissionLow();
		talonLeftA.configPeakCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_PEAK, 0);
		talonLeftA.configPeakCurrentDuration(Constants.DRIVE_CURRENT_LIMIT_PEAK_TIME, 0);
		talonLeftA.configContinuousCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_SUSTAINED, 0);

		talonRightA.configPeakCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_PEAK, 0);
		talonRightA.configPeakCurrentDuration(Constants.DRIVE_CURRENT_LIMIT_PEAK_TIME, 0);
		talonRightA.configContinuousCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_SUSTAINED, 0);

		talonLeftA.enableCurrentLimit(true);
		talonRightA.enableCurrentLimit(true);

		x = 0;
		y = 0;
		theta = 0;
		new Thread(() -> {
			while (true) {
				x += -1 * Math.cos(Math.toRadians(ahrs_gyro.getAngle())) * talonNativeToFPS(
						((talonLeftA.getSelectedSensorVelocity(0) + talonRightA.getSelectedSensorVelocity(0)) / 2));
				y += -1 * Math.sin(Math.toRadians(ahrs_gyro.getAngle())) * talonNativeToFPS(
						((talonLeftA.getSelectedSensorVelocity(0) + talonRightA.getSelectedSensorVelocity(0)) / 2));
				theta = Math.toRadians(ahrs_gyro.getAngle());
			}
		}).start();
	}

	private void initPID(double pid_id) {
		if (pid_id == high_pid_id) {
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
		} else {
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

	}

	public double[] getEncoderVelocities() {
		double l = talonLeftA.getSelectedSensorVelocity(timeout_ms);
		double r = talonRightA.getSelectedSensorVelocity(timeout_ms);
		return new double[] { l, r };
	}

	public double[] getEncoderPositions() {
		double l = talonLeftA.getSelectedSensorPosition(timeout_ms);
		double r = talonRightA.getSelectedSensorPosition(timeout_ms);
		return new double[] { l, r };
	}

	public void setDrivePower(double left, double right) {
		// Set the left and right motor power
		talonLeftA.set(ControlMode.PercentOutput, left);
		talonRightA.set(ControlMode.PercentOutput, right);
	}

	public void setVelocityTargets(double left, double right) {
		double multiplier = 600 * Constants.DRIVE_TRANSMISSION_RATIO;
		if (transmission_in_low) {
			multiplier = 600;
		}
		// System.out.println(" " + multiplier + " " + left + " " + right);
		talonLeftA.set(ControlMode.Velocity, left * 3);
		talonRightA.set(ControlMode.Velocity, right * 3);
	}

	public void setFPS(double left, double right) {
		if (left > LOW_GEAR_TOP_SPEED || right > LOW_GEAR_TOP_SPEED) {
			setTransmissionHigh();
		}

		if (left < LOW_GEAR_TOP_SPEED && right < LOW_GEAR_TOP_SPEED) {
			setTransmissionLow();
		}

		talonLeftA.set(ControlMode.Velocity, FPSToTalonNative(left));
		talonRightA.set(ControlMode.Velocity, FPSToTalonNative(right));
	}

	public double FPSToTalonNative(double fps) {
		return (fps * 1 * 1 * 4 * Constants.DRIVE_ENCODER_TICKS) / (1 * 6 * Math.PI * 12 * 1 * 100);
	}

	public void setPositionTargetIncrements(double leftOffset, double rightOffset) {
		double lTarget = talonLeftA.get() + leftOffset;
		double rTarget = talonRightA.get() + rightOffset;
		talonLeftA.set(ControlMode.Position, lTarget);
		talonRightA.set(ControlMode.Position, rTarget);
	}

	public void setTransmissionLow() {
		System.out.println("Transmission in Low Gear");
		transmission_in_low = true;
		initPID(low_pid_id);
		transmissiom_solenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void setTransmissionHigh() {
		System.out.println("Transmission in High Gear");
		transmission_in_low = false;
		initPID(high_pid_id);
		transmissiom_solenoid.set(DoubleSolenoid.Value.kReverse);

	}

	public double pidErrorAverage() {
		double average = (talonLeftA.getClosedLoopError(timeout_ms) + talonRightA.getClosedLoopError(timeout_ms)) / 2;
		return average;

	}

	public void zeroEncoderPositions() {
		talonLeftA.setSelectedSensorPosition(0, high_pid_id, timeout_ms);
		talonRightA.setSelectedSensorPosition(0, high_pid_id, timeout_ms);
		talonLeftA.set(ControlMode.Position, 0);
		talonRightA.set(ControlMode.Position, 0);

	}

	public void zero_gyro() {
		System.out.print("Zeroing Gyro...");
		ahrs_gyro.reset();
		System.out.println("...Zeroing Complete");
	}

	public double getGyroAngle() {
		// System.out.print(ahrs_gyro.getAngle());
		// System.out.print(" ");
		// System.out.println(ahrs_gyro.getCompassHeading());
		return ahrs_gyro.getAngle();

	}

	public double talonNativeToFPS(double something) {
		return something * 2.55663465 / 24850.407;
		// return (something) * (1/(4*128)) * (6*Math.PI) * (1/12) * 1000;
	}

	public double[] getOdometry() {
		double[] odo = new double[3];
		odo[0] = x;
		odo[1] = y;
		odo[2] = -theta % (Math.PI * 2);
		return odo;
	}

	public AHRS getGyro() {
		return ahrs_gyro;
	}
}
