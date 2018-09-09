package frc.team3863.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;

/**
 *
 */
public class AutoIntake extends Command {
    int state = 0;
    int counter = 0;
    boolean isComplete = false;

    public AutoIntake() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        state = 0;
        isComplete = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        switch (state) {
            case 0:
                if (Robot.kIntake.triIsCubeInIntake()) {  //Robot already has a cube
                    state = 6;
                    System.out.println("We already have a cube!");
                } else {
                    System.out.println("Looking to pick up a cube");
                    Robot.kElevator.goToPreset(0);     // Robot is waiting for a cube
                    Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_IDLE_POWER);
                    Robot.kIntake.openClaw();

                    state = 4;
                }
                break;

            case 4:
                if (counter > 20) {
                    state = 1;
                    counter = 0;
                    System.out.println("Counter complete");
                } else {
                    counter += 1;
                }
                break;

            case 1:
                if (Robot.kIntake.triIsCubeAngled()) {  //Robot sees an angled cube
                    state = 2;
                    System.out.println("Picking up an angled cube");
                    Robot.kIntake.setSkewedIntakePower(-Constants.INTAKE_MOTOR_POWER);
                } else if (Robot.kIntake.triIsCubeStraight()) { //Robot sees a straight cube
                    System.out.println("Picking up a straight cube");
                    Robot.kIntake.closeClaw();
                    state = 2;
                }
                break;

            case 2:
                Robot.kIntake.setIntakeWheelPower(-Constants.INTAKE_MOTOR_POWER);
                if (Robot.kIntake.triIsCubeInIntake()) {
                    state = 3;
                    Robot.kIntake.closeClaw();
                    System.out.println("Cube registered as in by sensors");
                }
                break;

            case 3:
                if (Robot.kIntake.testMotorCurrentThreshold(9.8) || Robot.kIntake.triIsCubeInIntake()) {
                    state = 5;
                    System.out.println("Intake motor current trip (this is good)");
                }

            case 5:
                Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_IDLE_POWER);
                Robot.kElevator.goToPreset(1);
                state = 6;

            case 6:
                isComplete = true;
                System.out.println("Auto cube intake complete!");
        }


    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isComplete;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_IDLE_POWER);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        state = 0;
        isComplete = true;
        Robot.kIntake.setIntakeWheelPower(Constants.INTAKE_IDLE_POWER);
    }
}
