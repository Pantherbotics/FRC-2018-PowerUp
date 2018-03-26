package org.usfirst.frc.team3863.robot.subsystems;

import org.usfirst.frc.team3863.robot.Constants;
import org.usfirst.frc.team3863.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

	WPI_TalonSRX winchTalon = new WPI_TalonSRX(RobotMap.CLIMBER_WINCH_TALON_ID);
	WPI_TalonSRX armTalon = new WPI_TalonSRX(RobotMap.CLIMBER_ARM_TALON_ID);
	private int armPidID = 0;
	private int timeout = 0;
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	public void init(){
		armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, armPidID, timeout);
		armTalon.config_kP(armPidID, Constants.HOOK_PID_P, timeout);
		armTalon.config_kI(armPidID, Constants.HOOK_PID_I, timeout);
		armTalon.config_kD(armPidID, Constants.HOOK_PID_D, timeout);
	}
	
	public void setWinchPower(double power){
		winchTalon.set(power);
	}
	
	public void setArmPos(double pos){
		armTalon.set(ControlMode.Position, pos);
	}
	
	public double getArmPos(){
		return armTalon.getSelectedSensorPosition(timeout);
	}

}
