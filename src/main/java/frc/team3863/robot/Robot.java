/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3863.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3863.robot.autonomous.*;
import frc.team3863.robot.commands.TransmissionHighGear;
import frc.team3863.robot.commands.ZeroLift;
import frc.team3863.robot.subsystems.Climber;
import frc.team3863.robot.subsystems.Drivetrain;
import frc.team3863.robot.subsystems.Elevator;
import frc.team3863.robot.subsystems.Intake;
import frc.team3863.robot.teleop.TeleopDualPartnerController;
import frc.team3863.robot.teleop.TeleopSinglePartnerController;
import frc.team3863.robot.util.Odometry;
import frc.team3863.robot.util.Units;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;


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
    //public static final Cameras kCameras = new Cameras();
    //Operator Interface instance. Contains button ==> command mappings
    public static OI m_oi = new OI();
    //PDP instance. Provides access to the robot's PDP, giving us current draw/breaker status info
    public static PowerDistributionPanel m_pdp = new PowerDistributionPanel();
    //DriverStation instance. Provides access to field/DS/match status and other info
    public DriverStation ds = DriverStation.getInstance();
    //True if autonomous command has been started (prevents starting it multiple times)
    public boolean is_auton_started = false;
    //Previous Message - Previous Driverstation Select
    String dsMessage = "";
    //Instance of the currently selected autonomous command
    PathedAutonomous m_autonomousCommand;
    //SmartDashboard dropdown menu for selecting autonomous modes.
    SendableChooser<Integer> m_chooser = new SendableChooser<>();
    //Instance of the currently selected teleop drive command
    Command m_teleopDriveCommand;
    //SmartDashboard dropdown menu for selecting teleop drive modes.
    SendableChooser<Command> m_drivechooser = new SendableChooser<Command>();
    //True if autonomous mode depends on FMS data (switch/scale position)
    boolean does_auto_need_field_data = false;
    //-1 if we want to use the DS position as the robot position,
    //otherwise 1,2,3 for L,C,R robot positions

    //Timer to limit number of SmartDashboard updates per second
    Timer timerInstance = new Timer();

    public static HashMap<String, Trajectory> paths;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     * <p>
     * Initalizes the Auton and Drivetrain SmartDashboard choosers, initalizes all
     * subsystem (that need initalization), and sets SmartDashboard defaults
     */
    @Override
    public void robotInit() {

        m_chooser.addDefault("Center Switch Auto (1-Cube)", 0);
        m_chooser.addObject("Left Scale Autonomous", 1);
        m_chooser.addObject("Right Scale Autonomous", 2);
        //Add options to the Auton Mode chooser, and add it to the SmartDashboard
        //The options are integers, accessed later via a switch statement.
        paths = collectPathsFromDirectory(Constants.PATH_LOCATION);



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

        //Enable the USB CameraServers
        //kCameras.enableCameras();

        kClimber.init();

        //Reset the SmartDashboard auton description
        SmartDashboard.putString("Auto Mode", "Auton Not Running");



        SmartDashboard.putData("Auto Mode", m_chooser);
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

        dsMessage = ds.getGameSpecificMessage();
        if(dsMessage.length()<3){
            does_auto_need_field_data = true;
        } else{
            m_autonomousCommand = selectAutonomous(m_chooser.getSelected(), dsMessage);
            kDrivetrain.setOdometry(m_autonomousCommand.getInitOdometry());
        }

        if (does_auto_need_field_data) {
            System.out.println("Waiting for field data...");
        }

        kDrivetrain.zeroGyro();
        Command zero = new ZeroLift();
        zero.start();
        Robot.kIntake.closeClaw();
        Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_IDLE_POWER);
        new TransmissionHighGear();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        if (does_auto_need_field_data) {
            dsMessage = ds.getGameSpecificMessage();
            boolean has_field_data;
            if(dsMessage.length() < 3) {
                has_field_data = false;
            } else{
                has_field_data = true;
            }//i have no idea what im doing -AF
            if (has_field_data) {
                m_autonomousCommand = selectAutonomous(m_chooser.getSelected(), dsMessage);
                kDrivetrain.setOdometry(m_autonomousCommand.getInitOdometry());
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

        //Command zero = new ZeroLift();
        //zero.start();

        Robot.kIntake.closeClaw();

        //Select the drive mode from the SmartDashboard
        m_teleopDriveCommand = m_drivechooser.getSelected();

        kDrivetrain.setDefaultCommand(m_teleopDriveCommand);
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

        SmartDashboard.putNumber("Left Velocity (ft/s)", Units.TalonNativeToFPS(vels[0]));
        SmartDashboard.putNumber("Right Velocity (ft/s)", Units.TalonNativeToFPS(vels[1]));
        //Add the Drivetrain L/R encoder positions to the SmartDashboard
        double[] poss = kDrivetrain.getEncoderPositions();
        //SmartDashboard.putNumber("Left Pos", poss[0]);
        //SmartDashboard.putNumber("Right Pos", poss[1]);

        //Add the elevator's target position, and actual position to the SmartDashboard
        SmartDashboard.putNumber("Elevator target", kElevator.target);
        SmartDashboard.putNumber("Elevator pos", kElevator.getPos());
        SmartDashboard.putBoolean("ElevatorLimit", kElevator.isLiftLowered());

        //Add the gyro angle
        //SmartDashboard.putNumber("Gyro", kDrivetrain.getGyroAngle());

        /*
        //Add the Ultrasonic and IR sensors (freq and voltage)
        SmartDashboard.putNumber("IntakeUltrasonic", kIntake.getAverageDistance());
        SmartDashboard.putNumber("IntakeLeftIR", kIntake.getLeftIR());
        SmartDashboard.putNumber("IntakeRightIR", kIntake.getRightIR());*/

        //Add the elevator's velocity
        SmartDashboard.putNumber("elevVelocity", kElevator.getVel());
        SmartDashboard.putNumber("Hook Arm Position", kClimber.getArmPos());

        Odometry odo = kDrivetrain.getOdometry();
        SmartDashboard.putNumber("Robot X Position", odo.getX());
        SmartDashboard.putNumber("Robot Y Position", odo.getY());
        SmartDashboard.putNumber("Robot Odometry (rad)", odo.getTheta());

    }

    public void outputPathsToDashboard(HashMap<String, Trajectory> paths, SendableChooser<String> chooser){
        System.out.println(paths.isEmpty());
        for(String key : paths.keySet()){
            System.out.println(key);
            chooser.addObject(key, key);
        }
    }

    public HashMap<String, Trajectory> collectPathsFromDirectory(String dir){
        HashMap<String, Trajectory> paths = new HashMap<>();
    
            ArrayList<File> filesInFolder = listf(dir);

            for(int i = filesInFolder.size()-1; i >=0 ; i--){
                File traj = filesInFolder.get(i);
                if (!traj.getName().contains("_source_Jaci.csv")){
                    filesInFolder.remove(i);
                }
            }
            for(File traj: filesInFolder){
                System.out.println(traj.getName());                                                                          //take all the File objects we just created & convert them into Trajectories to put into HashMap
                paths.put(traj.getName().replace("_source_Jaci.csv", ""), Pathfinder.readFromCSV(traj));
            }
            return paths;
    }


    public static ArrayList<File> listf(String directoryName) {
        File directory = new File(directoryName);

        // get all the files from a directory
        File[] fList = directory.listFiles();
        ArrayList<File> resultList = new ArrayList<File>(Arrays.asList(fList));
        for (File file : fList) {
            if (file.isFile()) {
                System.out.println(file.getAbsolutePath());
            } else if (file.isDirectory()) {
                resultList.addAll(listf(file.getAbsolutePath()));
            }
        }
        //System.out.println(fList);
        return resultList;
    }

    public PathedAutonomous selectAutonomous(int id, String fieldData){
        PathedAutonomous autonCommand = null;
        if(id == 0){
            if(fieldData.charAt(0) == 'L')
                autonCommand = new AutoCenterLeftOneCube();
            else
                autonCommand = new AutoCenterRightOneCube();
        } else if(id == 1){
            if(fieldData.charAt(1) == 'L')
                autonCommand = new AutoLeftSameTwoCube();
            else
                autonCommand = new AutoLeftDiffOneCube();
        } else if(id == 2){
            if(fieldData.charAt(2)=='L')
                autonCommand = new AutoRightDiffOneCube();
            else
                autonCommand = new AutoRightSameTwoCube();
        }
        return autonCommand;
    }

}
