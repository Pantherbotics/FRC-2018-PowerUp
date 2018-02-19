package org.usfirst.frc.team3863.robot.subsystems;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Cameras extends Subsystem {
    CameraServer camserver = CameraServer.getInstance();

    public void initDefaultCommand() {
    }
    
    public void enableCameras() {
    	camserver.startAutomaticCapture(0);
    	camserver.startAutomaticCapture(1);
    }
}

