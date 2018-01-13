/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3863.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	//Drivetrain CAN IDs
	public static int TALON_DRIVE_LEFTA_ID = 4;
	public static int TALON_DRIVE_LEFTB_ID = 5;
	public static int TALON_DRIVE_RIGHTA_ID = 1;
	public static int TALON_DRIVE_RIGHTB_ID = 3;
}
