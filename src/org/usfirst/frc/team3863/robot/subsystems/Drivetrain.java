package org.usfirst.frc.team3863.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.RobotMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
/**
 Controls the four CANTalons dedicated to the Drivetrain
 */
public class Drivetrain extends Subsystem {
	
	//ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    AHRS ahrs_gyro = new AHRS(I2C.Port.kOnboard);

	WPI_TalonSRX talonLeftA = new WPI_TalonSRX(RobotMap.TALON_DRIVE_LEFTA_ID);
	WPI_TalonSRX talonLeftB = new WPI_TalonSRX(RobotMap.TALON_DRIVE_LEFTB_ID);
	WPI_TalonSRX talonRightA = new WPI_TalonSRX(RobotMap.TALON_DRIVE_RIGHTA_ID);
	WPI_TalonSRX talonRightB = new WPI_TalonSRX(RobotMap.TALON_DRIVE_RIGHTB_ID);
	
	DoubleSolenoid transmissiom_solenoid = new DoubleSolenoid(RobotMap.PCM_TRANSMISSION_LOW, RobotMap.PCM_TRANSMISSION_HIGH);
		
	int pid_id = 0;
	public int timeout_ms = 0;
	
		
	public boolean transmission_in_low = true;
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void init() {
    	talonLeftA.setInverted(true);
    	talonLeftB.setInverted(true);
    	
    	talonLeftA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pid_id, timeout_ms);
    	talonLeftA.setSensorPhase(true);
    	
    	talonRightA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pid_id, timeout_ms);
    	talonRightA.setSensorPhase(true);
    	
    	talonLeftB.follow(talonLeftA);
    	talonRightB.follow(talonRightA);
    	
    	talonLeftA.configContinuousCurrentLimit(Constants.DRIVE_CURRENT_LIMIT, timeout_ms);
    	talonLeftB.configContinuousCurrentLimit(Constants.DRIVE_CURRENT_LIMIT, timeout_ms);
    	talonRightA.configContinuousCurrentLimit(Constants.DRIVE_CURRENT_LIMIT, timeout_ms);
    	talonRightB.configContinuousCurrentLimit(Constants.DRIVE_CURRENT_LIMIT, timeout_ms);
    	
    	initPID(1);
    	
    	zero_gyro();
    	
    	setTransmissionLow();
    }
    
    private void initPID(double multiplier) {
    	talonLeftA.configAllowableClosedloopError(0, pid_id, timeout_ms); 
    	talonLeftA.config_kF(pid_id, Constants.DRIVE_PID_F * multiplier, timeout_ms);
    	talonLeftA.config_kP(pid_id, Constants.DRIVE_PID_P * multiplier, timeout_ms);
    	talonLeftA.config_kI(pid_id, Constants.DRIVE_PID_I * multiplier, timeout_ms);
    	talonLeftA.config_kD(pid_id, Constants.DRIVE_PID_D * multiplier, timeout_ms);
        
    	talonRightA.configAllowableClosedloopError(0, pid_id, timeout_ms); 
    	talonRightA.config_kF(pid_id, Constants.DRIVE_PID_F * multiplier, timeout_ms);
    	talonRightA.config_kP(pid_id, Constants.DRIVE_PID_P * multiplier, timeout_ms);
    	talonRightA.config_kI(pid_id, Constants.DRIVE_PID_I * multiplier, timeout_ms);
    	talonRightA.config_kD(pid_id, Constants.DRIVE_PID_D * multiplier, timeout_ms);
    	
    }
    
    public double[] getEncoderVelocities() {
    	double l = talonLeftA.getSelectedSensorVelocity(timeout_ms);
    	double r = talonRightA.getSelectedSensorVelocity(timeout_ms);
    	return new double[] {l, r};
    }
    
    public double[] getEncoderPositions() {
    	double l = talonLeftA.getSelectedSensorPosition(timeout_ms);
    	double r = talonRightA.getSelectedSensorPosition(timeout_ms);
    	return new double[] {l, r};
    }
    
    public void setDrivePower(double left, double right) {
    	//Set the left and right motor power 
    	talonLeftA.set(ControlMode.PercentOutput, left);
    	talonRightA.set(ControlMode.PercentOutput, right);
    }
    
    public void setVelocityTargets(double left, double right) {
    	double multiplier = 600 * Constants.DRIVE_TRANSMISSION_RATIO;
    	if (transmission_in_low) {
    		multiplier = 600;
    	}
    	//System.out.println(" " + multiplier + " " + left + " " + right);
    	talonLeftA.set(ControlMode.Velocity, left * multiplier * 3);
    	talonRightA.set(ControlMode.Velocity, right * multiplier * 3);
    }
    
    public void setPositionTargetIncrements(double leftOffset, double rightOffset) {
    	double lTarget = talonLeftA.getSelectedSensorPosition(timeout_ms) + leftOffset;
    	double rTarget = talonRightA.getSelectedSensorPosition(timeout_ms) + rightOffset;
    	talonLeftA.set(ControlMode.Position, lTarget);
    	talonRightA.set(ControlMode.Position, rTarget);
    }
    
    public void setTransmissionLow() {
    	System.out.println("Transmission in Low Gear");
    	transmission_in_low = true;
    	initPID(1);
    	transmissiom_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
	public void setTransmissionHigh() {
		System.out.println("Transmission in High Gear");
		transmission_in_low = false;
		initPID(1/Constants.DRIVE_TRANSMISSION_RATIO);
		transmissiom_solenoid.set(DoubleSolenoid.Value.kReverse);
		
		
	}
	
	public double pidErrorAverage() {
		double average = (talonLeftA.getClosedLoopError(timeout_ms)+ talonRightA.getClosedLoopError(timeout_ms))/2;
		return average;
		
	}
	
	public void zero_gyro() {
		System.out.print("Zeroing Gyro...");
		ahrs_gyro.reset();
		System.out.println("...Zeroing Complete");
	}
	
	public double getGyroAngle() {
		//System.out.print(ahrs_gyro.getAngle());
		//System.out.print(" ");
		//System.out.println(ahrs_gyro.getCompassHeading());
		return ahrs_gyro.getAngle();
		
	}
	
    
}

