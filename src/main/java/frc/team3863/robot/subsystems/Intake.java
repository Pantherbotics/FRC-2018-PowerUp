package frc.team3863.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3863.robot.Constants;
import frc.team3863.robot.RobotMap;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class Intake extends Subsystem {
    public WPI_TalonSRX leftIntakeTalon = new WPI_TalonSRX(RobotMap.LEFT_INTAKE_TALON_ID);
    public WPI_TalonSRX rightIntakeTalon = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE_TALON_ID);
    public int timeout_ms = 0;
    List<Double> pastDistances = new ArrayList<Double>();
    Counter ultrasonicCounter = new Counter(RobotMap.SENSOR_INTAKE_ULTRASONIC);
    AnalogInput leftIRSensor = new AnalogInput(0);
    AnalogInput rightIRSensor = new AnalogInput(1);
    DoubleSolenoid claw_solenoid = new DoubleSolenoid(RobotMap.PCM_INTAKE_RETRACT, RobotMap.PCM_INTAKE_EXTEND);
    int pid_id = 0;
    boolean claw_closed;

    public void init() {
        //leftIntakeTalon.configContinuousCurrentLimit(Constants.INTAKE_CURRENT_LIMIT, timeout_ms);
        //leftIntakeTalon.enableCurrentLimit(true);
        //rightIntakeTalon.configContinuousCurrentLimit(Constants.INTAKE_CURRENT_LIMIT, timeout_ms);
        //rightIntakeTalon.enableCurrentLimit(true);
        //leftIntakeTalon.enableVoltageCompensation(true);
        //rightIntakeTalon.enableVoltageCompensation(true);

        ultrasonicCounter.setSemiPeriodMode(false);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    public void setIntakeWheelPower(double power) {
        if (power == 0) {
            power = -0.1;
        }
        System.out.println(power);
        leftIntakeTalon.set(ControlMode.PercentOutput, power);
        rightIntakeTalon.set(ControlMode.PercentOutput, -power);
    }

    public void closeClaw() {
        claw_solenoid.set(DoubleSolenoid.Value.kForward);
        claw_closed = true;
    }

    public void openClaw() {
        claw_solenoid.set(DoubleSolenoid.Value.kReverse);
        claw_closed = false;
    }

    public double getRawDistance() {
        double dist = (ultrasonicCounter.getPeriod() * 3 / -0.000147) + 992;
        pastDistances.add(dist);
        if (pastDistances.size() > 10) {
            pastDistances.remove(0);
        }
        return dist;
    }

    public double getAverageDistance() {
        getRawDistance();
        return pastDistances.stream().mapToDouble(val -> val).average().getAsDouble();
    }

    public boolean isCubeInIntake() {
        return (getAverageDistance() < 1.0);

    }

    public boolean isCubeInfrontOfIntake() {
        double dist = getAverageDistance();
        return (dist < Constants.AUTO_INTAKE_MAX_DIST && dist > 1.0);
    }

    public boolean testMotorCurrentThreshold(double amps) {
        return (leftIntakeTalon.getOutputCurrent() + rightIntakeTalon.getOutputCurrent() / 2) > amps;
    }


    public double getLeftIR() {
        return leftIRSensor.getVoltage();
    }

    public double getRightIR() {
        return rightIRSensor.getVoltage();
    }

    //TODO: Remove Magic Numbers, they are evil!!!
    private boolean isIRCubeEmpty(double voltage) {
        //0.2 = open
        //1.5 = cube in
        // 2.5 = cube angle / cube present
        return (Math.abs(voltage - 0.2) < 0.3);
    }

    private boolean isIRCubeInIntake(double voltage) {
        return (Math.abs(voltage - 1.5) < 0.3);
    }

    private boolean isIRCubeInfrontOfIntake(double voltage) {
        return (Math.abs(voltage - 2.5) < 0.3);
    }

    public boolean triIsCubeAngled() {
        boolean leftIR = isIRCubeInfrontOfIntake(getLeftIR());
        boolean rightIR = isIRCubeInfrontOfIntake(getRightIR());
        boolean center = isCubeInIntake();
        return (leftIR && center && rightIR);
    }

    public boolean triIsCubeStraight() {
        boolean leftIR = isIRCubeInfrontOfIntake(getLeftIR());
        boolean rightIR = isIRCubeInfrontOfIntake(getRightIR());
        boolean center = isCubeInfrontOfIntake();
        return (leftIR && center && rightIR && !claw_closed);
    }

    public boolean triIsCubeInIntake() {
        boolean leftIR = isIRCubeInIntake(getLeftIR());
        boolean rightIR = isIRCubeInIntake(getRightIR());
        boolean center = isCubeInIntake();
        return (leftIR && center && rightIR);
    }

    public void setSkewedIntakePower(double power) {
        leftIntakeTalon.set(ControlMode.PercentOutput, power * 1.5);
        rightIntakeTalon.set(ControlMode.PercentOutput, -power * 0.7);
    }
}

