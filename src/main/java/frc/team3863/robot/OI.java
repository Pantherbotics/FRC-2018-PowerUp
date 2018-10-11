/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3863.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team3863.robot.commands.*;

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

    public JoystickButton partnerLeftBumper = new JoystickButton(partnerController, 5);
    public JoystickButton partnerRightBumper = new JoystickButton(partnerController, 6);

    public JoystickButton partnerControlA = new JoystickButton(partnerController, 7);
    public JoystickButton partnerControlB = new JoystickButton(partnerController, 8);

    public JoystickButton partnerShare = new JoystickButton(partnerController, 9);
    public JoystickButton partnerOptions = new JoystickButton(partnerController, 10);

    public JoystickButton partnerTrackpad = new JoystickButton(partnerController, 14);

    public Joystick auxPartnerController = new Joystick(3);

    public JoystickButton auxPartnerX = new JoystickButton(auxPartnerController, 1);
    public JoystickButton auxPartnerA = new JoystickButton(auxPartnerController, 2);
    public JoystickButton auxPartnerB = new JoystickButton(auxPartnerController, 3);
    public JoystickButton auxPartnerY = new JoystickButton(auxPartnerController, 4);

    public JoystickButton auxPartnerLeftBumper = new JoystickButton(auxPartnerController, 5);
    public JoystickButton auxPartnerRightBumper = new JoystickButton(auxPartnerController, 6);

    public JoystickButton auxPartnerLeftTrigger = new JoystickButton(auxPartnerController, 7);
    public JoystickButton auxPartnerRightTrigger = new JoystickButton(auxPartnerController, 8);

    public JoystickButton auxPartnerBack = new JoystickButton(auxPartnerController, 9);
    public JoystickButton auxPartnerStart = new JoystickButton(auxPartnerController, 10);


    /**
     * Button mappings that should be enabled for all drive modes
     */
    public OI() {

    }

    /**
     * Button mappings that should be enabled when driving with a single joystick
     * This method is called when the DriveSingleJoystick command starts
     */
    public void initSingleJoystick() {
        leftJoystickTrigger.whenPressed(new ToggleTransmissionState());
    }

    /**
     * Button mappings that should be enabled when driving with the partner controller
     * This method is called when the DriveController command starts
     */
    public void initSinglePartnerController() {
        partnerButtonX.whenPressed(new ToggleTransmissionState());
        partnerButtonY.whileHeld(new MoveLiftBy(Constants.ELEVATOR_DRIVE_INCREMENT));
        partnerButtonA.whileHeld(new MoveLiftBy(Constants.ELEVATOR_DRIVE_DECREMENT));
        partnerLeftBumper.whenPressed(new OuttakeCube());
        //partnerLeftBumper.whileHeld(new EnableIntakeWheels(false)); //Wheels run in forward direction
        //partnerRightBumper.whileHeld(new EnableIntakeWheels(true)); //Wheels run in reverse direction
        partnerControlA.whenPressed(new IntakeClaw(true)); //open claw
        partnerControlB.whenPressed(new IntakeClaw(false)); //close claw
        //partnerShare.whileHeld(new RaiseLeftRamp());
        //partnerOptions.whileHeld(new RaiseRightRamp());
        partnerTrackpad.whenPressed(new AutoIntake());
    }

    public void initDualPartnerController() {
        partnerLeftBumper.whenPressed(new ToggleTransmissionState());
        partnerRightBumper.whenPressed(new ToggleTransmissionState());

        partnerControlA.whenPressed(new TransmissionLowGear());
        partnerControlB.whenPressed(new TransmissionHighGear());

        partnerButtonX.whileHeld(new RunClimber());
        partnerButtonY.whenPressed(new RaiseHook());
        partnerButtonA.whenPressed(new LowerHook());

        partnerShare.whenPressed(new RotateDegrees(180));
        auxPartnerY.whenPressed(new OuttakeCube());
        auxPartnerA.whenPressed(new AutoIntake());
        //auxPartnerX.whenPressed(new RaiseLeftRamp());
        //auxPartnerB.whenPressed(new RaiseRightRamp());

        auxPartnerLeftTrigger.whenPressed(new IntakeClaw(true));
        auxPartnerRightTrigger.whenPressed(new IntakeClaw(false));

        auxPartnerLeftBumper.whileHeld(new EnableIntakeWheels(true));
        auxPartnerRightBumper.whileHeld(new EnableIntakeWheels(false));

        auxPartnerStart.whenPressed(new ZeroLift());
    }

}
