package org.firstinspires.ftc.teamcode.DecodeNationals;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Intake Subsystem for Nationals
 * Uses a REV UltraPlanetary motor with 3:1 ratio.
 * PIDF velocity control with voltage compensation for consistent speed.
 */
@Configurable
public class IntakeSubsystem implements Subsystem {

    public static final IntakeSubsystem INSTANCE = new IntakeSubsystem();

    private IntakeSubsystem() {}

    /* ---------------- Hardware ---------------- */
    private DcMotorEx intakeMotor;
    private VoltageSensor voltageSensor;

    /* ---------------- PIDF Configuration ---------------- */
    public static double Kp = 0.0005;
    public static double Ki = 0.0001;
    public static double Kd = 0.00001;
    public static double Kf = 0.00028;
    public static double Ks = 0.05;

    /* ---------------- Voltage Compensation ---------------- */
    public static double nominalVoltage = 13.4;
    public static double maxVelocity = 2000.0; // Max ticks/sec at nominal voltage - tune this

    /* ---------------- Speed Configuration (ticks/sec) ---------------- */
    public static double INTAKE_VELOCITY = 2000.0;
    public static double OUTTAKE_VELOCITY = -1400.0;

    /* ---------------- Motor Configuration ---------------- */
    public static boolean MOTOR_REVERSED = true;

    /* ---------------- State ---------------- */
    private boolean isRunning = false;
    private double targetVelocity = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    // Debug/telemetry values
    public double debugCurrentVelocity = 0.0;
    public double debugTargetVelocity = 0.0;
    public double debugError = 0.0;
    public double debugOutput = 0.0;

    /* ---------------- Initialization ---------------- */

    public void initialize(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        timer.reset();
        integralSum = 0.0;
        lastError = 0.0;
        isRunning = false;
        targetVelocity = 0.0;
    }

    /* ---------------- Periodic Update ---------------- */

    @Override
    public void periodic() {
        if (intakeMotor == null) return;

        if (isRunning) {
            updatePIDFLoop();
        }

        debugTargetVelocity = targetVelocity;
    }

    private void updatePIDFLoop() {
        double currentVelocity = intakeMotor.getVelocity();
        double currentVoltage = getCurrentVoltage();

        // Voltage-compensated clamping
        double voltageRatio = currentVoltage / nominalVoltage;
        double maxAchievable = maxVelocity * voltageRatio;
        double clampedTarget = Math.signum(targetVelocity) * Math.min(Math.abs(targetVelocity), maxAchievable);

        // PID calculations
        double error = clampedTarget - currentVelocity;
        double dt = timer.seconds();
        if (dt == 0) dt = 0.001;

        double derivative = (error - lastError) / dt;

        if (Math.abs(clampedTarget) > 0) {
            integralSum += error * dt;
            integralSum = Math.max(-0.5, Math.min(0.5, integralSum));
        } else {
            integralSum = 0.0;
        }

        double pidOutput = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        // Feedforward
        double feedforward = 0.0;
        if (Math.abs(clampedTarget) > 0) {
            feedforward = Math.signum(clampedTarget) * Ks + (clampedTarget * Kf);
        }

        double totalOutput = (pidOutput + feedforward) * getVoltageCompensation();
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        intakeMotor.setPower(totalOutput);

        lastError = error;
        timer.reset();

        debugCurrentVelocity = currentVelocity;
        debugError = error;
        debugOutput = totalOutput;
    }

    /* ---------------- Utility Methods ---------------- */

    private double getVoltageCompensation() {
        return nominalVoltage / getCurrentVoltage();
    }

    private double getCurrentVoltage() {
        if (voltageSensor != null) {
            try {
                double voltage = voltageSensor.getVoltage();
                if (voltage > 0) return voltage;
            } catch (Exception e) {
                // Fall through
            }
        }
        return nominalVoltage;
    }

    /* ---------------- Core Methods ---------------- */

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        isRunning = Math.abs(velocity) > 0.01;
        if (isRunning) {
            integralSum = 0.0;
            lastError = 0.0;
            timer.reset();
        } else {
            intakeMotor.setPower(0.0);
        }
    }

    public void runIntake() {
        setTargetVelocity(INTAKE_VELOCITY);
    }

    public void runOuttake() {
        setTargetVelocity(OUTTAKE_VELOCITY);
    }

    public void stopIntake() {
        setTargetVelocity(0.0);
    }

    /* ---------------- Commands ---------------- */

    public Command intake() {
        return new InstantCommand(this::runIntake).requires(this);
    }

    public Command outtake() {
        return new InstantCommand(this::runOuttake).requires(this);
    }

    public Command stop() {
        return new InstantCommand(this::stopIntake).requires(this);
    }

    public Command toggle() {
        return new InstantCommand(() -> {
            if (isRunning) {
                stopIntake();
            } else {
                runIntake();
            }
        }).requires(this);
    }

    /* ---------------- Getters ---------------- */

    public boolean isRunning() {
        return isRunning;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }
}
