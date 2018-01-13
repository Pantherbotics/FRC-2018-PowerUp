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
import org.usfirst.frc.team3863.robot.commands.BaselineAuto;
import org.usfirst.frc.team3863.robot.commands.SwitchFarLeftAuto;
import org.usfirst.frc.team3863.robot.commands.SwitchNearLeftAuto;
import org.usfirst.frc.team3863.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static final Drivetrain kDrivetrain = new Drivetrain();
	public static OI m_oi = new OI();
	public static PowerDistributionPanel m_pdp = new PowerDistributionPanel();
	public DriverStation ds = DriverStation.getInstance();
	public Alliance alliance = ds.getAlliance();
	boolean auton_right;

	Command m_autonomousCommand;
	SendableChooser<Integer> m_chooser = new SendableChooser<Integer>();
	
	public void updateSmartDashboard() {
		
		String msg = ds.getGameSpecificMessage();
		
		boolean isBlue = (alliance == Alliance.Blue);
		SmartDashboard.putBoolean("OurAlliance", isBlue);
		if (msg.length() < 3) {
			System.out.println("Malformed Field Data: "+msg);
		}else {
			SmartDashboard.putBoolean("OurSwitch_L", msg.charAt(0) == 'L' ^ !isBlue); //True = Blue; False = Red;
			SmartDashboard.putBoolean("Scale_L", msg.charAt(1) == 'L' ^ !isBlue);
			SmartDashboard.putBoolean("EnemySwitch_L", msg.charAt(2) == 'L' ^ !isBlue);  
			
			SmartDashboard.putBoolean("OurSwitch_R", msg.charAt(0) == 'R' ^ !isBlue); //True = Blue; False = Red;
			SmartDashboard.putBoolean("Scale_R", msg.charAt(1) == 'R' ^ !isBlue);
			SmartDashboard.putBoolean("EnemySwitch_R", msg.charAt(2) == 'R' ^ !isBlue);   
		}
		if (m_autonomousCommand != null) {
			SmartDashboard.putData("AutonCommand", m_autonomousCommand);
		}
        
        if (auton_right) {
			SmartDashboard.putString("AutonSide", "Right");
		}else {
			SmartDashboard.putString("AutonSide", "Left");
		}
        
        //SmartDashboard.putData("PDP", m_pdp);
	}
	
	/**
	 * Update the DriverStation and auton init code with the given Auton command
	 */
	public void updateAuton() {
		int ds_choice = m_chooser.getSelected();
		switch(ds_choice) { 
		 	case 1: 						//Determine Auto mode from switch positions
		 		DriverStation ds = DriverStation.getInstance();
				String msg = ds.getGameSpecificMessage();
				int loc = ds.getLocation();
				auton_right = (loc==3);     //Invert the auton side if we are in the right driverstation
				if(msg.charAt(0) == 'L') {		 
					m_autonomousCommand = new SwitchNearLeftAuto(auton_right);
		        } else if (msg.charAt(0) == 'R'){ //Our switch is to the Right
		        	m_autonomousCommand = new SwitchFarLeftAuto(auton_right);
		        }
				break;
		 	case 2: 						//Baseline Auto
		 		m_autonomousCommand = new BaselineAuto(); 
		 		break;
		 	default:
		 		m_autonomousCommand = null;
		 		break; 
	 	}
		
	}
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		m_chooser.addDefault("None", 0);
		m_chooser.addObject("AutoSelect Switch", 1);
		m_chooser.addObject("Baseline", 2);
		SmartDashboard.putData("Auto mode", m_chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
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
		updateSmartDashboard();
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
		Scheduler.getInstance().run();
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
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		updateSmartDashboard();
	}
}
