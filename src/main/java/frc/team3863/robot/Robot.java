/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3863.robot;

import java.io.File;

import edu.wpi.first.wpilibj.TimedRobot;
import jaci.pathfinder.Trajectory;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Pathfinder;
import frc.team3863.robot.autonomous.AutoBaseline;
import frc.team3863.robot.autonomous.AutoBaselineOpenLoop;
import frc.team3863.robot.autonomous.AutoLeftScale;
import frc.team3863.robot.autonomous.AutoLeftSwitchCenter;
import frc.team3863.robot.autonomous.AutoLeftSwitchFar;
import frc.team3863.robot.autonomous.AutoLeftSwitchNear;
import frc.team3863.robot.autonomous.AutoPathPlanningTest;
import frc.team3863.robot.autonomous.Paths;
import frc.team3863.robot.commands.ZeroLift;
import frc.team3863.robot.subsystems.Cameras;
import frc.team3863.robot.subsystems.Climber;
import frc.team3863.robot.subsystems.Drivetrain;
import frc.team3863.robot.subsystems.Elevator;
import frc.team3863.robot.subsystems.Intake;
import frc.team3863.robot.subsystems.Ramps;
import frc.team3863.robot.teleop.TeleopDualPartnerController;
import frc.team3863.robot.teleop.TeleopSingleJoystick;
import frc.team3863.robot.teleop.TeleopSinglePartnerController;

import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
    //Instance of the Drivetrain subsystem. Controls wheels + transmissions. Accessed by commands + code in the main loop
    public static final Drivetrain kDrivetrain = new Drivetrain();
    //Instance of the Elevator subsystem. Controls elevator lift motor.
    public static final Elevator kElevator = new Elevator();
    //Instance of the Intake subsystem. Controls Intake wheels + servo arm.
    public static final Intake kIntake = new Intake();

    public static final Climber kClimber = new Climber();

    //Previous Message - Previous Driverstation Select
    String PrevMsg = "";
    int PrevDsSelect;

    public static final Ramps kRamps = new Ramps();

    public static final Cameras kCameras = new Cameras();


    //Operator Interface instance. Contains button ==> command mappings
    public static OI m_oi = new OI();
    //PDP instance. Provides access to the robot's PDP, giving us current draw/breaker status info
    public static PowerDistributionPanel m_pdp = new PowerDistributionPanel();
    //DriverStation instance. Provides access to field/DS/match status and other info
    public DriverStation ds = DriverStation.getInstance();

    //Instance of the currently selected autonomous command
    Command m_autonomousCommand;
    //SmartDashboard dropdown menu for selecting autonomous modes.
    SendableChooser<Integer> m_chooser = new SendableChooser<Integer>();

    //Instance of the currently selected teleop drive command
    Command m_teleopDriveCommand;
    //SmartDashboard dropdown menu for selecting teleop drive modes.
    SendableChooser<Command> m_drivechooser = new SendableChooser<Command>();

    //True if autonomous command has been started (prevents starting it multiple times)
    public boolean is_auton_started = false;
    //True if autonomous mode depends on FMS data (switch/scale position)
    boolean does_auto_need_field_data = false;
    //-1 if we want to use the DS position as the robot position,
    //otherwise 1,2,3 for L,C,R robot positions
    int override_ds_loc = -1;

    //Timer to limit number of SmartDashboard updates per second
    Timer timerInstance = new Timer();

    /**
     * Updates the SmartDashboard with robot state + debug info
     */
    public void updateSmartDashboard() {

        //Get the Switch status from the Field Management System (FMS)
        String msg = ds.getGameSpecificMessage();

        //Check if we are on the Blue alliance
        boolean isBlue = (ds.getAlliance() == Alliance.Blue);
        //Add alliance state to the SmartDashboard
        SmartDashboard.putBoolean("OurAlliance", isBlue);

        if (msg.length() < 3) {
            //Error out if the Field data is not in an expected format
            //System.out.println("Malformed Field Data: "+msg);
        } else {
            //Add Switch, Scale, Switch data to the SmartDashboard (True = Blue; False = Red)
            // ^ !isBlue will invert the output (reverse the POV when our driver station is on the opposite side of the field)
            SmartDashboard.putBoolean("OurSwitch_L", msg.charAt(0) == 'L' ^ !isBlue);
            SmartDashboard.putBoolean("Scale_L", msg.charAt(1) == 'L' ^ !isBlue);
            SmartDashboard.putBoolean("EnemySwitch_L", msg.charAt(2) == 'L' ^ !isBlue);

            //Add right side of switch data (the inverse of the left side)
            SmartDashboard.putBoolean("OurSwitch_R", msg.charAt(0) == 'R' ^ !isBlue);
            SmartDashboard.putBoolean("Scale_R", msg.charAt(1) == 'R' ^ !isBlue);
            SmartDashboard.putBoolean("EnemySwitch_R", msg.charAt(2) == 'R' ^ !isBlue);
        }

        //Add the autonomous command to the SmartDashboard if it exists
        if (m_autonomousCommand != null) {
            SmartDashboard.putData("AutonCommand", m_autonomousCommand);
        }


        //Add the transmission state to the SmartDashboard
        if (kDrivetrain.transmission_in_low) {
            SmartDashboard.putString("Transmission", "Low Gear");
        } else {
            SmartDashboard.putString("Transmission", "High Gear");
        }

        //Add the PDP (Power Distrubution Panel) data to the SmartDashboard
        SmartDashboard.putData("PDP", m_pdp);

        //Get the drivetrain encoder velocities, and add them to the SmartDashboard
        double[] vels = kDrivetrain.getEncoderVelocities();
        //SmartDashboard.putNumber("Left Velocity (Native)", vels[0]);
        //SmartDashboard.putNumber("Right Velocity (Native)", vels[1]);

        SmartDashboard.putNumber("Left Velocity (ft/s)", Robot.kDrivetrain.talonNativeToFPS(vels[0]));
        SmartDashboard.putNumber("Right Velocity (ft/s)", Robot.kDrivetrain.talonNativeToFPS(vels[1]));
        //Add the Drivetrain L/R encoder positions to the SmartDashboard
        double[] poss = kDrivetrain.getEncoderPositions();
        SmartDashboard.putNumber("Left Pos", poss[0]);
        SmartDashboard.putNumber("Right Pos", poss[1]);

        //Add the elevator's target position, and actual position to the SmartDashboard
        SmartDashboard.putNumber("Elevator target", kElevator.target);
        SmartDashboard.putNumber("Elevator pos", kElevator.getPos());
        SmartDashboard.putBoolean("ElevatorLimit", kElevator.isLiftLowered());

        //Add the gyro angle
        SmartDashboard.putNumber("Gyro", kDrivetrain.getGyroAngle());

        //Add the Ultrasonic and IR sensors (freq and voltage)
        SmartDashboard.putNumber("IntakeUltrasonic", kIntake.getAverageDistance());
        SmartDashboard.putNumber("IntakeLeftIR", kIntake.getLeftIR());
        SmartDashboard.putNumber("IntakeRightIR", kIntake.getRightIR());

        //Add the elevator's velocity
        SmartDashboard.putNumber("elevVelocity", kElevator.getVel());
        SmartDashboard.putNumber("Hook Arm Position", kClimber.getArmPos());

        SmartDashboard.putNumber("Robot X Position", kDrivetrain.getOdometry()[0]);
        SmartDashboard.putNumber("Robot Y Position", kDrivetrain.getOdometry()[1]);
        SmartDashboard.putNumber("Robot Heading (rad)", kDrivetrain.getOdometry()[2]);

    }

    /**
     * Determine which autonomous mode should run (based on SmartDashboard selection AND field state)
     */
    public boolean runAutoWithFieldData(int force_ds_location) {
        //Get the FMS field data (example: "LRL"
        // L <== Our switch scoring position
        // R <== Center scale scoring position
        // L <== Opponent's switch scoring position
        String msg = ds.getGameSpecificMessage();

        //Don't process anything because we don't have valid field data yet
        //Tell autonomousPeriodic that we are still waiting for FMS data
        if (msg.length() < 3) {
            return false;
        }

        //Will store Robot/Driverstation location (1,2,3 for Left, Center, Right)
        int ds_loc;

        //If we want to override the DS/robot location, set ds_loc to the
        //override value, otherwise get the DS/robot location from the field
        if (force_ds_location > 0) {
            ds_loc = force_ds_location;
            System.out.println("Overriding DS Location (field: " + ds.getLocation() + ")");
        } else {
            ds_loc = ds.getLocation();
        }
        System.out.println("We are in DS #" + ds_loc);

        int ds_choice = m_chooser.getSelected();

        //Test both the robot and goal locations
        boolean is_ds_right_side = (ds_loc == 3);
        boolean is_goal_right_side = (msg.charAt(0) == 'R');
        boolean is_scale_goal_right_side = (msg.charAt(1) == 'R');

        if (ds_choice == 7 || ds_choice == 8) {
            if (is_scale_goal_right_side == is_ds_right_side) {
                System.out.println("AUTON: BLESS THE RNG!!!!!!!!");
                if (ds_choice == 7) {
                    System.out.println("AUTON: Left Scale");
                    // Left scale auto, runs left side of field only.
                    // Should only run when scale is on left side
                    m_autonomousCommand = new AutoLeftScale(false);
                    m_autonomousCommand.start();
                    return true;

                } else if (ds_choice == 8) {
                    System.out.println("AUTON: Right Scale");
                    // Right scale auto, runs right side of field only.
                    // Should only run when scale is on right
                    m_autonomousCommand = new AutoLeftScale(true);
                    m_autonomousCommand.start();
                    return true;
                }
            } else {
                System.out.println("AUTON: GOD DAMN RNG!!");
                m_autonomousCommand = null;
                return true;
            }

        }

        //Center auto for robot in center of field
        if (ds_loc == 2) {
            if (is_goal_right_side) {
                System.out.println("AUTON: Center Right Switch");
            } else {
                System.out.println("AUTON: Center Left Switch");
            }
            // Center switch auto, runs left side of field by default.
            // When is_goal_right_side is true, the auto mode will
            // go to the right side
            m_autonomousCommand = new AutoLeftSwitchCenter(is_goal_right_side);
            m_autonomousCommand.start();
            return true;
        }

        //Near auto when robot and goal are on the same side
        if (is_goal_right_side == is_ds_right_side) {
            if (is_ds_right_side) {
                System.out.println("AUTON: Near Right Switch");
            } else {
                System.out.println("AUTON: Near Left Switch");
            }
            // Near switch auto, runs left side of field by default.
            m_autonomousCommand = new AutoLeftSwitchNear(is_ds_right_side);
            m_autonomousCommand.start();
            return true;

            //Far auto when robot and goal are on opposite sides
        } else if (is_goal_right_side != is_ds_right_side) {
            if (is_ds_right_side) {
                System.out.println("AUTON: Far Right Switch");
            } else {
                System.out.println("AUTON: Far Left Switch");
            }
            // Far switch auto, runs left side of field by default.
            m_autonomousCommand = new AutoLeftSwitchFar(is_ds_right_side);
            m_autonomousCommand.start();
            return true;
        } else {
            System.out.println("MALFORMED FIELD DATA: " + msg);
            return false;
        }
    }

    /**
     * Returns true if auto requires field data
     */
    boolean selectAuto() {
        int ds_choice = m_chooser.getSelected();
        switch (ds_choice) {
            //Automatically determine robot location from DS location, and run switch auto
            case 1:
                System.out.println("Automatic DS switch auto mode selected");
                override_ds_loc = -1;
                return true;

            //Force robot to left side of field, and run switch auto
            case 4:
                System.out.println("Override Left switch auto mode selected");
                override_ds_loc = 1;
                return true;

            //Force robot to right side of field, and run switch auto
            case 5:
                System.out.println("Override Right switch auto mode selected");
                override_ds_loc = 3;
                return true;

            //Force robot to Center side of field, and run switch auto
            case 6:
                System.out.println("Override Center switch auto mode selected");
                override_ds_loc = 2;
                return true;

            case 7:
                System.out.println("Override Left half scale auto mode selected");
                override_ds_loc = 1;
                return true;

            case 8:
                System.out.println("Override Right half scale auto mode selected");
                override_ds_loc = 3;
                return true;

            //Run Baseline with PID control
            case 2:
                m_autonomousCommand = new AutoBaseline();
                System.out.println("PID Baseline auto mode selected");
                return false;

            //Run Baseline without PID control (OpenLoop)
            case 3:
                m_autonomousCommand = new AutoPathPlanningTest();
                System.out.println("Path Planning Test!");
                return false;

            //No auton selected
            default:
                m_autonomousCommand = null;
                return false;
        }
    }


    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     * <p>
     * Initalizes the Auton and Drivetrain SmartDashboard choosers, initalizes all
     * subsystem (that need initalization), and sets SmartDashboard defaults
     */
    @Override
    public void robotInit() {

        //Add options to the Auton Mode chooser, and add it to the SmartDashboard
        //The options are integers, accessed later via a switch statement.
        m_chooser.addDefault("None", 0);

        m_chooser.addObject("AutoSelect Score Switch", 1);
        m_chooser.addObject("Left Score Switch", 4);
        m_chooser.addObject("Center Score Switch", 6);
        m_chooser.addObject("Right Score Switch", 5);
        m_chooser.addObject("Left Half Score Scale", 7);
        m_chooser.addObject("Right Half Score Scale", 8);
        m_chooser.addObject("[PID] Baseline", 2);
        m_chooser.addObject("[PWR] Baseline", 3);
        SmartDashboard.putData("Auto mode", m_chooser);

        //Add options to the Drive Mode chooser, and add it to the SmartDashboard
        //The options are instances of the given drive commands,
        m_drivechooser.addDefault("[PID] Dual Partner Controller", new TeleopDualPartnerController(true));
        m_drivechooser.addObject("[PID] Single Partner Controller", new TeleopSinglePartnerController(true));
        m_drivechooser.addObject("[PWR] Single Partner Controller", new TeleopSinglePartnerController(false));
        m_drivechooser.addObject("[PWR] Dual Partner Controller", new TeleopDualPartnerController(false));
        SmartDashboard.putData("Teleop Drive mode", m_drivechooser);

        //Initialize the Drivetrain subsystem, and add to the SmartDashboard
        kDrivetrain.init();
        SmartDashboard.putData("Drivetrain", kDrivetrain);

        kIntake.init();
        SmartDashboard.putData("Intake", kIntake);

        //Initialize the Elevator subsystem and add to the SmartDashboard
        kElevator.initPID();
        SmartDashboard.putData("Elevator", kElevator);

        //Initialize Ramp subsystem (set to default safe state)
        kRamps.init();

        //Enable the USB CameraServers
        kCameras.enableCameras();

        kClimber.init();

        //Reset the SmartDashboard auton description
        SmartDashboard.putString("Autosomis Mode", "Auton Not Running");


        System.out.println("Generating Paths");
        String name = "testTraj";
        boolean check = new File(name).exists();
        Trajectory traj;

        if (check) {
            File pathFile = new File(name + ".traj");
            traj = Pathfinder.readFromFile(pathFile);
        } else {
            Trajectory.Config configuration = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, .005, 12, 4, 20);
            traj = Pathfinder.generate(Paths.CenterSwitch, configuration);
        }
        System.out.println("finished");
        Paths.addTraj(traj);
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {

        //Stop the auton command if it is running
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        //Stop the teleop drive command if it is running
        if (m_teleopDriveCommand != null) {
            m_teleopDriveCommand.cancel();
        }
    }

    /**
     * Re-calculate autonomous selection and update the smartdashboard when disabled
     */
    @Override
    public void disabledPeriodic() {
        //Let the scheduler process any running commands
        Scheduler.getInstance().run();

        //Update the SmartDashboard with debug + state info
        updateSmartDashboard();

        //Calculate which autonomous to run
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        //Update the SmartDashboard with debug + state info
        //updateSmartDashboard();
        is_auton_started = false;
        does_auto_need_field_data = false;
        override_ds_loc = -1;

        does_auto_need_field_data = selectAuto();
        if (does_auto_need_field_data) {
            System.out.println("Waiting for field data...");
        }

        kDrivetrain.zero_gyro();

        Command zero = new ZeroLift();
        zero.start();
        Robot.kIntake.closeClaw();
        Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_IDLE_POWER);

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        if (does_auto_need_field_data) {
            boolean has_field_data = runAutoWithFieldData(override_ds_loc);
            if (has_field_data) {
                does_auto_need_field_data = false;
                System.out.println("Auton field data registered");
            }
        }

        if (m_autonomousCommand != null && !is_auton_started) {
            is_auton_started = true;
            System.out.println("Start Auton");
            m_autonomousCommand.start();
        }

        Scheduler.getInstance().run();

        double t = Timer.getFPGATimestamp();

        if (Math.floor(t * 1000) % 2 == 0) {
            //Update the SmartDashboard with debug + state info
            updateSmartDashboard();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        Command zero = new ZeroLift();
        zero.start();

        Robot.kRamps.init();
        Robot.kIntake.closeClaw();

        //Select the drive mode from the SmartDashboard
        m_teleopDriveCommand = m_drivechooser.getSelected();

        // schedule the drive command
        if (m_teleopDriveCommand != null) {
            m_teleopDriveCommand.start();
        }

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        //Let the scheduler process any running commands
        Scheduler.getInstance().run();

        double t = Timer.getFPGATimestamp();

        if (Math.floor(t * 1000) % 2 == 0) {
            //Update the SmartDashboard with debug + state info
            updateSmartDashboard();
        }
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

        String Zero = "1";
        //System.out.println(Zero);

        double t = Timer.getFPGATimestamp();

        if (Math.floor(t * 1000) % 2 == 0) {
            //Update the SmartDashboard with debug + state info
            updateSmartDashboard();
        }
    }

}
