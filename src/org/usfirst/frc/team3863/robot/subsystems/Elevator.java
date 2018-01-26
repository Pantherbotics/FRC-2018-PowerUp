package org.usfirst.frc.team3863.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	ControlMode mode = ControlMode.Position;
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
    	elevDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pid_id, timeout_ms);
    	elevDriveTalon.setSensorPhase(false);
    	elevDriveTalon.configNominalOutputForward(0, timeout_ms);
    	elevDriveTalon.configNominalOutputReverse(0, timeout_ms);
    	elevDriveTalon.configPeakOutputForward(1, timeout_ms);
    	elevDriveTalon.configPeakOutputReverse(-1, timeout_ms);

    	elevDriveTalon.configAllowableClosedloopError(0, pid_id, timeout_ms); 

    	elevDriveTalon.config_kF(pid_id, 0.0, timeout_ms);
    	elevDriveTalon.config_kP(pid_id, 1.1, timeout_ms);
        elevDriveTalon.config_kI(pid_id, 0.0, timeout_ms);
        elevDriveTalon.config_kD(pid_id, 0.0, timeout_ms);
    }
    
    public void setTargetPosition(int new_target) {
        target = new_target;
    	elevDriveTalon.set(mode, new_target);
    	System.out.println("Elevator Target Pos: " + new_target + " | pos: " + elevDriveTalon.getSelectedSensorPosition(timeout_ms));
    }
    
    //Used by driving control systems to set speed limits. 
    public boolean isLiftRaised() {
    	return false;
    }
    
    
}

