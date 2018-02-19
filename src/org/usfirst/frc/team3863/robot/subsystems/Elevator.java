package org.usfirst.frc.team3863.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
/**
 *
 */
public class Elevator extends Subsystem {
	
	public int target = 0;

	public WPI_TalonSRX elevDriveTalon = new WPI_TalonSRX(RobotMap.ELEVATOR_DRIVE_TALON_ID);
	int pid_id = 0;
	public int timeout_ms = 0;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public double getPos() {
    	return elevDriveTalon.getSelectedSensorPosition(timeout_ms);
    }
    
    //Initalize PID settings
    public void initPID() {
    	setTargetPosition(0);
    	elevDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pid_id, timeout_ms);
    	elevDriveTalon.setSensorPhase(false);
    	elevDriveTalon.configNominalOutputForward(0, timeout_ms);
    	elevDriveTalon.configNominalOutputReverse(0, timeout_ms);
    	elevDriveTalon.configPeakOutputForward(1, timeout_ms);
    	elevDriveTalon.configPeakOutputReverse(-1, timeout_ms);
    	elevDriveTalon.configContinuousCurrentLimit(Constants.ELEVATOR_CURRENT_LIMIT, timeout_ms);

    	elevDriveTalon.configAllowableClosedloopError(0, pid_id, timeout_ms); 

    	elevDriveTalon.config_kF(pid_id, Constants.ELEVATOR_PID_F, timeout_ms);
    	elevDriveTalon.config_kP(pid_id, Constants.ELEVATOR_PID_P, timeout_ms);
        elevDriveTalon.config_kI(pid_id, Constants.ELEVATOR_PID_I, timeout_ms);
        elevDriveTalon.config_kD(pid_id, Constants.ELEVATOR_PID_D, timeout_ms);
        
        elevDriveTalon.configForwardSoftLimitThreshold(Constants.ELEVATOR_SOFT_LIMIT, timeout_ms);
        elevDriveTalon.configForwardSoftLimitEnable(true, timeout_ms);
        
        setTargetPosition(elevDriveTalon.getSelectedSensorPosition(timeout_ms));
    }
    
    public void setTargetPosition(int new_target) {
    	if (new_target > Constants.ELEVATOR_SOFT_LIMIT) {
    		new_target = Constants.ELEVATOR_SOFT_LIMIT;
    	}else if (new_target < 50) {
    		new_target = 50;
    	}
        target = new_target;
    	elevDriveTalon.set(ControlMode.Position, new_target);
    }
    
    public void setMotorPower(double power) {
    	elevDriveTalon.set(ControlMode.PercentOutput, power);
    }
    
    //Used by driving control systems to set speed limits. 
    public boolean isLiftRaised() {
    	return (elevDriveTalon.getSelectedSensorPosition(timeout_ms) > 100);
    }
    
    public boolean isLiftLowered() {
    	return elevDriveTalon.getSensorCollection().isRevLimitSwitchClosed();
    }
    
    public boolean testMotorCurrentThreshold(double amps) {
    	return elevDriveTalon.getOutputCurrent() > amps;
    }

	public void zeroEncoder() {
		target = 0;
		elevDriveTalon.setSelectedSensorPosition(0, pid_id, timeout_ms);
		System.out.println("Elevator encoder zeroed!");
	}
	
	public void goToPreset(int presetID) {
		if (presetID == 0) {
			//zeroEncoder();
			setTargetPosition(Constants.ELEVATOR_PRESETS[presetID]);
		}else {
			setTargetPosition(Constants.ELEVATOR_PRESETS[presetID]);
		}
		
	}
	
	public double getHeightPercent() {
		return (getPos() / Constants.ELEVATOR_SOFT_LIMIT));
	}
    
    
}

