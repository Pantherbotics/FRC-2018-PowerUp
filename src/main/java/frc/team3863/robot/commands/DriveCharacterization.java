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
        System.out.println("Starting Drive Characterization");
        sb = new StringBuilder();
        sb.append("voltage,left velocity,right velocity\n");
        try {
            pw = new PrintWriter(new File("/home/lvuser/CharacterizationResults.csv"));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {


        Robot.kDrivetrain.setDrivePower(power, power);
        System.out.println(power * 12.0 + "," + Robot.kDrivetrain.getEncoderVelocities()[0] + "," + Robot.kDrivetrain.getEncoderVelocities()[1] + "\n");
        sb.append(power * 12.0);
        sb.append(",");
        sb.append(Robot.kDrivetrain.getEncoderVelocities()[0]);
        sb.append(",");
        sb.append(Robot.kDrivetrain.getEncoderVelocities()[1]);
        sb.append("\n");

        if (Math.abs(timer % 0.5) < 0.01) {
            power += 0.04166666666; //0.5v per 0.5s
        }

        timer += .02;

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        System.out.println("Time: " + timer);
        return Math.abs(timer - (12-.02)) < 0.01;
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("Finished!");
        Robot.kDrivetrain.setDrivePower(0, 0);
        pw.write(sb.toString());
        pw.close();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.kDrivetrain.setDrivePower(0, 0);
        pw.write(sb.toString());
        pw.close();
    }
}
