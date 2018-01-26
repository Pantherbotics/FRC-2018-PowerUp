/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3863.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import org.usfirst.frc.team3863.robot.commands.DecrementLift;
import org.usfirst.frc.team3863.robot.commands.IncrementLift;
import org.usfirst.frc.team3863.robot.commands.ToggleTransmissionState;;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	
	public Joystick leftJoystick = new Joystick(0);
	public Joystick rightJoystick = new Joystick(1);
	public JoystickButton leftJoystickTrigger = new JoystickButton(leftJoystick, 1);
	public JoystickButton rightJoystickTrigger = new JoystickButton(rightJoystick, 1);
	
	public Joystick partnerController = new Joystick(2);
	public JoystickButton partnerButtonX = new JoystickButton(partnerController, 1);
	public JoystickButton partnerButtonB = new JoystickButton(partnerController, 3);
	public JoystickButton partnerButtonA = new JoystickButton(partnerController, 2);
	public JoystickButton partnerButtonY = new JoystickButton(partnerController, 4);
	
	public OI() {
		
	}
	
	public void initDriveSingleJoystick() {
		leftJoystickTrigger.whenPressed(new ToggleTransmissionState());
	}
	
	public void initDriveController() {
		partnerButtonX.whenPressed(new ToggleTransmissionState());
		partnerButtonY.whileHeld(new IncrementLift());
		partnerButtonA.whileHeld(new DecrementLift());
	}

}
