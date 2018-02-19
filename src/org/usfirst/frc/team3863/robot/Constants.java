package org.usfirst.frc.team3863.robot;

public class Constants {
	//Drivetrain ticks/rev
	public static int DRIVE_ENCODER_TICKS = 1440;  //400: practice; 1440: competition 
	public static double DRIVE_WHEEL_DIAMETER = 18.850;
	public static double DRIVE_TRANSMISSION_RATIO = 2.4; //high speed is 2.4 times faster than low speed
	
	//Drivetrain ticks/rev
	public static int LIFT_ENCODER_TICKS = 100;
	
	//multiplier for error in RotateDegrees
	//TODO: TUNE ME!!!!
	public static double DRIVE_ROTATE_P = 0.5;
	
	//Drivetrain PID 
	public static double DRIVE_PID_F = 0.0;
	public static double DRIVE_PID_P = 0.55;
	public static double DRIVE_PID_I = 0.000;
	public static double DRIVE_PID_D = 0.00;
	
	//Maximum current draw for each drivetrain CANTalon
	public static int DRIVE_CURRENT_LIMIT = 28;
	
	//Elevator PID
	public static double ELEVATOR_PID_F = 0.0;
	public static double ELEVATOR_PID_P = 2.0;
	public static double ELEVATOR_PID_I = 0.003;
	public static double ELEVATOR_PID_D = 0.0;
	
	//Maximum current draw for Elevator CANTalon
	public static int ELEVATOR_CURRENT_LIMIT = 35;
	
	//Software limit for elevator height
	public static int ELEVATOR_SOFT_LIMIT = 5450;
	
	//Manual increment/decrements for elevator control
	public static int ELEVATOR_DRIVE_INCREMENT = 60;
	public static int ELEVATOR_DRIVE_DECREMENT = -60;
	
	//percent-output for intake motors (when enabled)
	public static double INTAKE_MOTOR_POWER = 0.75;
	
	public static int INTAKE_CURRENT_LIMIT = 15;
	
	//Deadband (dead-zone) for various input devices. 
	public static double JOYSTICK_DEADBAND = 0.05;
	public static double CONTROLLER_DEADBAND = 0.05;
	
	public static double AUTO_INTAKE_MAX_DIST = 17;
	
	public static int[] ELEVATOR_PRESETS = {50,    //Bottom 
			                                500,  //Slightly Raised (off the ground)
	                                        800,  //HumanPlayer station
	                                        1000, //Switch
	                                        5000, //Scale
	                                        5600};//Max Travel
 	
}