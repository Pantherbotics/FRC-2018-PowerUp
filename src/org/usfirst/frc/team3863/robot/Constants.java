package org.usfirst.frc.team3863.robot;

public class Constants {
	//Drivetrain ticks/rev
	public static int DRIVE_ENCODER_TICKS = 400;
	public static double DRIVE_WHEEL_DIAMETER = 18.850;
	
	//Drivetrain ticks/rev
	public static int LIFT_ENCODER_TICKS = 100;
	
	//Drivetrain PID 
	public static double DRIVE_PID_F = 0.0;
	public static double DRIVE_PID_P = 2.7;
	public static double DRIVE_PID_I = 0.05;
	public static double DRIVE_PID_D = -0.8;
	
	//Maximum current draw for each drivetrain CANTalon
	public static int DRIVE_CURRENT_LIMIT = 30;
	
	//Elevator PID
	public static double ELEVATOR_PID_F = 0.0;
	public static double ELEVATOR_PID_P = 1.2;
	public static double ELEVATOR_PID_I = 0.0000000001;
	public static double ELEVATOR_PID_D = 0.0;
	
	//Maximum current draw for Elevator CANTalon
	public static int ELEVATOR_CURRENT_LIMIT = 8;
	
	//Software limit for elevator height
	public static int ELEVATOR_SOFT_LIMIT = 5600;
	
	//Manual increment/decrements for elevator control
	public static int ELEVATOR_DRIVE_INCREMENT = 60;
	public static int ELEVATOR_DRIVE_DECREMENT = 60;
	
	//percent-output for intake motors (when enabled)
	public static double INTAKE_MOTOR_POWER = 0.25;
	
	//Deadband (dead-zone) for various input devices. 
	public static double JOYSTICK_DEADBAND = 0.05;
	public static double CONTROLLER_DEADBAND = 0.05;
	
}