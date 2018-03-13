package org.usfirst.frc.team3863.robot;

public class Constants {
	//Drivetrain ticks/rev
	//TODO: CHANGE ME FOR COMPETITION ROBOT!!!!!!!!!!!!!!!!
	//public static int DRIVE_ENCODER_TICKS = 1440;  //400: practice; 1440: competition 
	public static int DRIVE_ENCODER_TICKS = 400;
	public static double DRIVE_WHEEL_DIAMETER = 18.850;
	public static double DRIVE_TRANSMISSION_RATIO = 2.4; //high speed is 2.4 times faster than low speed
	
	//Drivetrain ticks/rev
	public static int LIFT_ENCODER_TICKS = 100;
	
	//multiplier for error in RotateDegrees
	//TODO: TUNE ME!!!!
	public static double DRIVE_ROTATE_P = 0.0111;
	
	//Seconds from neutral to full ramp
	public static double DRIVE_RAMP_SECONDS = 0.2;
	
	//Drivetrain PID 
	public static double DRIVE_PID_F = 0.0;
	public static double DRIVE_PID_P = 0.60;
	public static double DRIVE_PID_I = 0.00001;
	public static double DRIVE_PID_D = 0.0001;
	
	//Maximum current draw for each drivetrain CANTalon
	public static int DRIVE_CURRENT_LIMIT = 28;
	
	//Elevator PID
	public static double ELEVATOR_PID_F = 0.8184; //Guaranteed random: calculated using a fair dice roll
	public static double ELEVATOR_PID_P = 2.1;
	public static double ELEVATOR_PID_I = 0.0000;
	public static double ELEVATOR_PID_D = 1.1;
	
	public static int ELEVATOR_PID_CRUISE_VEL = 400;
	public static int ELEVATOR_PID_ACCELERATION = 400;
	
	//Maximum current draw for Elevator CANTalon
	public static int ELEVATOR_CURRENT_LIMIT = 35;
	
	//Software limit for elevator height
	//TODO: Comp robot changes
	//public static int ELEVATOR_SOFT_LIMIT = 5450; //COMP ROBOT
	public static int ELEVATOR_SOFT_LIMIT = 5000; //Practice Robot
	
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
	
	public static int[] ELEVATOR_PRESETS = {5,    //Bottom 
			                                500,  //Slightly Raised (off the ground)
	                                        800,  //HumanPlayer station
	                                        1750, //Switch
	                                        5000, //Scale
	                                        5450};//Max Travel
	
	//Ramp servo angles
	public static double RAMP_SERVO_LEFT_LATCH_ANGLE = 90;
	public static double RAMP_SERVO_RIGHT_LATCH_ANGLE = 90;
	public static double RAMP_SERVO_RIGHT_OPEN_ANGLE = -90;
	public static double RAMP_SERVO_LEFT_OPEN_ANGLE = -170;
 	
}