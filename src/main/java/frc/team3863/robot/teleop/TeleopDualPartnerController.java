package frc.team3863.robot.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3863.robot.Constants;
import frc.team3863.robot.Robot;
import frc.team3863.robot.commands.ElevatorSetpoint;
import frc.team3863.robot.util.CheesyDriveHelper;
import frc.team3863.robot.util.DriveSignal;

/**
 *
 */
public class TeleopDualPartnerController extends Command {
    boolean usepid;
    double lastPRY = -20;
    Integer lastPOV;
    CheesyDriveHelper cheesyDriveHelper;

    public TeleopDualPartnerController(boolean usePIDDrive) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.kDrivetrain);
        usepid = usePIDDrive;
        cheesyDriveHelper = new CheesyDriveHelper();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        System.out.println("(COMP) Dual Partner Controller Drive enabled");
        Robot.m_oi.initDualPartnerController();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double twist = Robot.m_oi.partnerController.getZ();
        double y = Robot.m_oi.partnerController.getY();
        double partnerY = Robot.m_oi.auxPartnerController.getY();
        double partnerRY = Robot.m_oi.auxPartnerController.getRawAxis(3);

        int auxPov = Robot.m_oi.auxPartnerController.getPOV();
        if (Math.abs(twist) <= Constants.CONTROLLER_DEADBAND) {
            twist = 0;
        }
        if (Math.abs(y) <= Constants.CONTROLLER_DEADBAND) {
            y = 0;
        }
        if (Math.abs(partnerY) <= Constants.CONTROLLER_DEADBAND) {
            partnerY = 0;
        }
        if (Math.abs(partnerRY) <= Constants.CONTROLLER_DEADBAND) {
            partnerRY = 0;
        }

        if (lastPOV == null || auxPov != lastPOV) {
            Command povCommand = null;
            switch (auxPov) {
                case 0:
                    povCommand = new ElevatorSetpoint(5); //Top
                    break;
                case 90:
                    povCommand = new ElevatorSetpoint(1); //Raised
                    break;
                case 180:
                    povCommand = new ElevatorSetpoint(0); //Bottom
                    break;
                case 270:
                    povCommand = new ElevatorSetpoint(3); //Switch
                    break;
            }
            if (povCommand != null) {
                povCommand.start();
            }
            lastPOV = auxPov;
        }

        if (Math.abs(partnerY) > 0.05) {
            int pos_increment = (int) Math.round(Constants.ELEVATOR_DRIVE_INCREMENT * -partnerY);
            Robot.kElevator.setTargetPosition(Robot.kElevator.target + pos_increment);
        }


        double elevDampen = 1.0 - Robot.kElevator.getHeightPercent();
        if (elevDampen < 0.8) {
            elevDampen = 0.8;
        }

        if (lastPRY == -20 || lastPRY != partnerRY) {
            if (partnerRY == 0 && Math.abs(lastPRY) > 0) {
                Robot.kIntake.setIntakeWheelPower(0);
                System.out.println("Partner Override Zero Intake");
            }
            if (Math.abs(partnerRY) != 0) {
                double sign = Math.signum(partnerRY);
                Robot.kIntake.setIntakeWheelPower(-1 * sign * Math.pow(partnerRY, 2));
            }
            lastPRY = partnerRY;
        }

        DriveSignal drive = cheesyDriveHelper.cheesyDrive(y, -twist, !Robot.kDrivetrain.transmission_in_low);

        if (usepid) {
            Robot.kDrivetrain.setFPS(drive.getLeft() * 23, drive.getRight() * 23);
        } else {
            Robot.kDrivetrain.setDrivePower(drive.getLeft(), drive.getRight());
        }

        if (Robot.m_oi.auxPartnerStart.get() && Robot.m_oi.auxPartnerBack.get()) {
            if (!Robot.kRamps.is_left_ramp_deployed) {
                Robot.kRamps.deployLeftRamp();
            }
            if (!Robot.kRamps.is_right_ramp_deployed) {
                Robot.kRamps.deployRightRamp();
            }
        }

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
