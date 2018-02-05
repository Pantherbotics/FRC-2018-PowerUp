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
	
	public static int DRIVE_CURRENT_LIMIT = 30;
	
	//Elevator PID
	public static double ELEVATOR_PID_F = 0.0;
	public static double ELEVATOR_PID_P = 1.2;
	public static double ELEVATOR_PID_I = 0.0000000001;
	public static double ELEVATOR_PID_D = 0.0;
	
	public static int ELEVATOR_CURRENT_LIMIT = 30;
	
	public static int ELEVATOR_SOFT_LIMIT = 5600;
	
}