package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Robot;

import java.io.File;
import java.io.PrintWriter;

public class DriveCharacterization extends Command {
    //remember: dt = .02s
    private double timer;
    private double power;
    private StringBuilder sb;
    private PrintWriter pw;
    public DriveCharacterization() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kDrivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        timer = 0;
        power = 0;
        sb = new StringBuilder();
        sb.append("voltage,left velocity,right velocity\n");
        try {
            pw = new PrintWriter(new File("/home/lvuser/CharacterizationResults.csv"));
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if(timer < 6){
            Robot.kDrivetrain.setDrivePower(power, 0);
            sb.append(power*12.0 + "," + Robot.kDrivetrain.getEncoderVelocities()[0] + ",0\n" );
        }else{
            Robot.kDrivetrain.setDrivePower(0, power-12.0);
            sb.append(power*12.0 + ",0,"+ Robot.kDrivetrain.getEncoderVelocities()[1] +"\n" );
        }

        if(timer %0.5 == 0){
            power +=0.04166666666; //0.5v per 0.5s
        }

        timer += .2;

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(timer-12) < 0.001;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.kDrivetrain.setDrivePower(0, 0);
        //Robot.kIntake.closeClaw();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        //Robot.kIntake.setIntakeWheelPower(-1);
    }
}
