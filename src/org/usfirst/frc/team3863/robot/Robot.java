/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3863.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import org.usfirst.frc.team3863.robot.autonomous.AutoBaseline;
import org.usfirst.frc.team3863.robot.autonomous.AutoFarSwitchScore;
import org.usfirst.frc.team3863.robot.autonomous.AutoNearSwitchScore;
import org.usfirst.frc.team3863.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3863.robot.subsystems.Elevator;
import org.usfirst.frc.team3863.robot.subsystems.Intake;
import org.usfirst.frc.team3863.robot.teleop.TeleopSingleJoystick;
import org.usfirst.frc.team3863.robot.teleop.TeleopSinglePartnerController;


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
	
	//Operator Interface instance. Contains button ==> command mappings
	public static OI m_oi = new OI();                    
	//PDP instance. Provides access to the robot's PDP, giving us current draw/breaker status info
	public static PowerDistributionPanel m_pdp = new PowerDistributionPanel(); 
	//DriverStation instance. Provides access to field/DS/match status and other info 
	public DriverStation ds = DriverStation.getInstance();         
	
	//The current alliance (Alliance.Blue || Alliance.Red) to which the robot is assigned. 
	public Alliance alliance = ds.getAlliance(); 
	//used in updateAuton() to indicate if the autonomous mode should run in 'right' mode (invert l/r values)
	boolean auton_right;                                            

	//Instance of the currently selected autonomous command 
	Command m_autonomousCommand;                      
	 //SmartDashboard dropdown menu for selecting autonomous modes. 
	SendableChooser<Integer> m_chooser = new SendableChooser<Integer>();
	
	//Instance of the currently selected teleop drive command  
	Command m_teleopDriveCommand;	     
	//SmartDashboard dropdown menu for selecting teleop drive modes. 
	SendableChooser<Command> m_drivechooser = new SendableChooser<Command>(); 
	
	/**
	 * Updates the SmartDashboard with robot state + debug info
	 */
	public void updateSmartDashboard() {
		
		//Get the Switch status from the Field Management System (FMS)
		String msg = ds.getGameSpecificMessage();
		
		//Check if we are on the Blue alliance
		boolean isBlue = (alliance == Alliance.Blue);
		//Add alliance state to the SmartDashboard
		SmartDashboard.putBoolean("OurAlliance", isBlue);
		
		if (msg.length() < 3) {
			//Error out if the Field data is not in an expected format
			//System.out.println("Malformed Field Data: "+msg);
		}else {
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
        
		//Add the auton left/right info to SmartDashboard
        if (auton_right) {
			SmartDashboard.putString("AutonSide", "Right");
		}else {
			SmartDashboard.putString("AutonSide", "Left");
		}
        
        //Add the transmission state to the SmartDashboard
        if (kDrivetrain.transmission_in_low) {
        	SmartDashboard.putString("Transmission", "Low Gear");
        }else {
        	SmartDashboard.putString("Transmission", "High Gear");
        }
        
        //Add the PDP (Power Distrubution Panel) data to the SmartDashboard
        SmartDashboard.putData("PDP", m_pdp);
        
        //Get the drivetrain encoder velocities, and add them to the SmartDashboard
        double[] vels = kDrivetrain.getEncoderVelocities();
        SmartDashboard.putNumber("Left Velocity", vels[0]);
        SmartDashboard.putNumber("Right Velocity", vels[1]);
        
        //Add the elevator's target position, and actual position to the SmartDashboard
        double[] poss = kDrivetrain.getEncoderPositions();
        SmartDashboard.putNumber("Left Pos", poss[0]);
        SmartDashboard.putNumber("Right Pos", poss[1]);
        
        SmartDashboard.putNumber("Elevator target", kElevator.target);
    	SmartDashboard.putNumber("Elevator pos", kElevator.getPos());
    	
    	SmartDashboard.putNumber("Gyro", kDrivetrain.getGyroAngle());
        
        
	}
	
	/**
	 * Determine which autonomous mode should run (based on SmartDashboard selection AND field state)
	 */
	public void updateAuton() {
		//Get the currently selected autonomous mode (represented as a number)
		int ds_choice = m_chooser.getSelected();
		
		switch(ds_choice) { 
			//Option 1: Automatically determine auton mode based on field status
		 	case 1:
		 		
		 		//Get the scoring positions of the switches from the FMS
		 		String msg = ds.getGameSpecificMessage();
		 		
		 		//Get our driverstation number (1-3, left-right)
				int loc = ds.getLocation();
				
				//True if we are in the rightmost driverstation (#3)
				auton_right = (loc==3);     
				
				//When our switch is on the Left side
				if(msg.charAt(0) == 'L') {		 

				//When our switch is on the right side
					m_autonomousCommand = new AutoNearSwitchScore(auton_right);
		        } else if (msg.charAt(0) == 'R'){ //Our switch is to the Right
		        	m_autonomousCommand = new AutoFarSwitchScore(auton_right);
		        }
				break;

			//Baseline Autonomous mode
		 	case 2: 						
		 		m_autonomousCommand = new AutoBaseline(); 
		 		break;
		 		
		 	//No autonomous mode (default)
		 	default:
		 		m_autonomousCommand = null;
		 		break; 
	 	}
		
	}
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 * 
	 * Initalizes the Auton and Drivetrain SmartDashboard choosers, initalizes all 
	 * subsystem (that need initalization), and sets SmartDashboard defaults
	 */
	@Override
	public void robotInit() {
		
		//Add options to the Auton Mode chooser, and add it to the SmartDashboard
		//The options are integers, accessed later via a switch statement. 
		m_chooser.addDefault("None", 0);
		m_chooser.addObject("AutoSelect Score Switch", 1);
		m_chooser.addObject("Baseline", 2);
		SmartDashboard.putData("Auto mode", m_chooser);
		
		//Add options to the Drive Mode chooser, and add it to the SmartDashboard
		//The options are instances of the given drive commands, 
		m_drivechooser.addDefault("Single Partner Controller", new TeleopSinglePartnerController());
		m_drivechooser.addObject("Single Joystick", new TeleopSingleJoystick());
		SmartDashboard.putData("Teleop Drive mode", m_drivechooser);
		
		//Initalize the Drivetrain subsystem, and add to the SmartDashboard
		kDrivetrain.init();
		SmartDashboard.putData("Drivetrain", kDrivetrain);
		
		//Initalize the Elevator subsystem and add to the SmartDashboard
		kElevator.initPID();
		SmartDashboard.putData("Elevator", kElevator);
		
		//Reset the SmartDashboard auton description
		SmartDashboard.putString("Autosomis Mode", "Auton Not Running");
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 * 
	 */
	@Override
	public void disabledInit() {

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
		updateAuton();
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
		
		kDrivetrain.zero_gyro();
		
		//Calculate which autonomous to run
		updateAuton();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		//Let the scheduler process any running commands 
		Scheduler.getInstance().run();
		
		//Update the SmartDashboard with debug + state info
		updateSmartDashboard();
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
		
		//Update the SmartDashboard with debug + state info
		updateSmartDashboard();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		//Update the SmartDashboard with debug + state info
		updateSmartDashboard();
	}
}
