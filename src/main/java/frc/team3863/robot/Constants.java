package frc.team3863.robot;

public class Constants {
    //Drivetrain ticks/rev
    //TODO: CHANGE ME FOR COMPETITION ROBOT!!!!!!!!!!!!!!!!
    public static final double DRIVE_ENCODER_TICKS = 512 * 1.5; //128 * 4
    //public static final int DRIVE_ENCODER_TICKS = 1440;  //400: practice; 1440: competition
    //public static int DRIVE_ENCODER_TICKS = 400;
    public static final double DRIVE_WHEEL_DIAMETER = 6;
    public static final double DRIVE_TRANSMISSION_RATIO = 2.4; //high speed is 2.4 times faster than low speed

    //Drivetrain ticks/rev
    public static final int LIFT_ENCODER_TICKS = 100;

    //multiplier for error in RotateDegrees
    //TODO: Increase to account for carpet
    public static final double DRIVE_ROTATE_P = 0.0067;

    //Seconds from neutral to full ramp
    public static final double DRIVE_RAMP_SECONDS = 0.2;

    //Drivetrain PID
    public static final double HIGH_DRIVE_PID_F = 1.328571;
    public static final double HIGH_DRIVE_PID_P = 2.5;
    public static final double HIGH_DRIVE_PID_I = 0.0000;
    public static final double HIGH_DRIVE_PID_D = 000;

    public static final double LOW_DRIVE_PID_F = 1023/770;
    public static final double LOW_DRIVE_PID_P = 1.5;
    public static final double LOW_DRIVE_PID_I = 0.0000;
    public static final double LOW_DRIVE_PID_D = 0.000;

    //Maximum current draw for each drivetrain CANTalon
    public static final int DRIVE_CURRENT_LIMIT_SUSTAINED = 36;
    public static final int DRIVE_CURRENT_LIMIT_PEAK = 80;
    public static final int DRIVE_CURRENT_LIMIT_PEAK_TIME = 10;  //ms

    //Elevator PID
    public static final double ELEVATOR_PID_F = 0.8184; //Guaranteed random: calculated using a fair dice roll
    public static final double ELEVATOR_PID_P = 2.1;
    public static final double ELEVATOR_PID_I = 0.0000;
    public static final double ELEVATOR_PID_D = 1.1;

    public static final int ELEVATOR_PID_CRUISE_VEL = 400;
    public static final int ELEVATOR_PID_ACCELERATION = 300;

    //Maximum current draw for Elevator CANTalon
    public static final int ELEVATOR_CURRENT_LIMIT = 35;

    //Software limit for elevator height
    //TODO: Comp robot changes
    //public static int ELEVATOR_SOFT_LIMIT = 5450; //COMP ROBOT
    public static final int ELEVATOR_SOFT_LIMIT = 5000; //Practice Robot

    //Manual increment/decrements for elevator control
    public static final int ELEVATOR_DRIVE_INCREMENT = 60;
    public static final int ELEVATOR_DRIVE_DECREMENT = -60;

    //percent-output for intake motors (when enabled)
    public static final double INTAKE_MOTOR_POWER = 0.75;


    //idle power for intake
    public static final double INTAKE_IDLE_POWER = -0.2;

    //Deadband (dead-zone) for various input devices.
    public static final double JOYSTICK_DEADBAND = 0.05;
    public static final double CONTROLLER_DEADBAND = 0.05;

    public static final double AUTO_INTAKE_MAX_DIST = 17;

    public static final int[] ELEVATOR_PRESETS = {5,    //Bottom
            500,  //Slightly Raised (off the ground)
            800,  //HumanPlayer station
            1850, //Switch
            5000, //Scale
            5450};//Max Travel

    //Ramp servo angles
    public static final double RAMP_SERVO_LEFT_LATCH_ANGLE = 90;
    public static final double RAMP_SERVO_RIGHT_LATCH_ANGLE = 90;
    public static final double RAMP_SERVO_RIGHT_OPEN_ANGLE = -90;
    public static final double RAMP_SERVO_LEFT_OPEN_ANGLE = -170;

    //Hook Delivery Arm PID
    public static final double HOOK_PID_P = 0.75;
    public static final double HOOK_PID_I = 0.0005;
    public static final double HOOK_PID_D = 0.10;

    public static final double HOOK_RAISED_POSITION = 1600;
    public static final double HOOK_LOWERED_POSITION = 10;

    public static final double WHEEL_BASE = 26 / 12;

    public static final String PATH_LOCATION = "/home/lvuser/paths";
}