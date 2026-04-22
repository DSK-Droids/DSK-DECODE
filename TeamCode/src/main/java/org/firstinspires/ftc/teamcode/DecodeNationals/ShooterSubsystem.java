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
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Shooter Subsystem for Nationals
 * Uses a PIDF control loop to maintain constant flywheel velocity with voltage compensation.
 * Two flywheel motors run in opposite directions for the shooter mechanism.
 */
@Configurable
public class ShooterSubsystem implements Subsystem {

    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

    private ShooterSubsystem() {}

    /* ---------------- Hardware ---------------- */
    private DcMotorEx leftFlyWheelMotor;
    private DcMotorEx rightFlyWheelMotor;
    private VoltageSensor voltageSensor;

    /* ---------------- PIDF Configuration ---------------- */
    // Tune these values for your specific shooter
    public static double flyWheelKp = 0.000;   // Proportional gain
    public static double flyWheelKi = 0.000;   // Integral gain
    public static double flyWheelKd = 0.0000;  // Derivative gain
    public static double flyWheelKf = 0.00035;  // Feedforward gain (velocity-based)
    public static double flyWheelKs = 0.08;     // Static friction compensation

    // Voltage compensation settings
    public static double nominalVoltage = 13.4;           // Reference voltage for tuning
    public static double maxVelocityAtNominalVoltage = 2400; // Max achievable velocity (ticks/sec)

    // Control settings
    public static double targetVelocity = 0.0;  // Target velocity in ticks/second
    public static double velocityTolerance = 50.0; // Tolerance for "at speed" detection

    /* ---------------- State ---------------- */
    private boolean isEnabled = false;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    // Telemetry/debug values
    public double debugCurrentVelocity = 0.0;
    public double debugTargetVelocity = 0.0;
    public double debugError = 0.0;
    public double debugOutput = 0.0;
    public double debugVoltage = 0.0;

    /* ---------------- Initialization ---------------- */

    /**
     * Must be called in onInit() to set up hardware
     */
    public void initialize(HardwareMap hardwareMap) {
        // Initialize motors
        leftFlyWheelMotor = hardwareMap.get(DcMotorEx.class, "leftFlyWheelMotor");
        rightFlyWheelMotor = hardwareMap.get(DcMotorEx.class, "rightFlyWheelMotor");

        // Configure left motor
        leftFlyWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFlyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFlyWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Configure right motor
        rightFlyWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFlyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Get voltage sensor
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        // Reset state
        timer.reset();
        integralSum = 0.0;
        lastError = 0.0;
        isEnabled = false;
    }

    /* ---------------- Periodic Update ---------------- */

    @Override
    public void periodic() {
        if (leftFlyWheelMotor == null || rightFlyWheelMotor == null) return;

        if (isEnabled) {
            updatePIDFLoop();
        }
    }

    /**
     * Core PIDF control loop with voltage compensation
     */
    private void updatePIDFLoop() {
        // Get current velocity (average of both motors)
        double leftVelocity = leftFlyWheelMotor.getVelocity();
        double rightVelocity = rightFlyWheelMotor.getVelocity();
        double currentVelocity = (leftVelocity + rightVelocity) / 2.0;

        // Get current voltage
        double currentVoltage = getCurrentVoltage();

        // Voltage compensation - adjust max achievable velocity based on current voltage
        double voltageRatio = currentVoltage / nominalVoltage;
        double maxAchievableVelocity = maxVelocityAtNominalVoltage * voltageRatio;
        double clampedTargetVelocity = Math.min(targetVelocity, maxAchievableVelocity);

        // Calculate error
        double error = clampedTargetVelocity - currentVelocity;

        // Calculate time delta
        double dt = timer.seconds();
        if (dt == 0) dt = 0.001; // Prevent division by zero

        // Derivative term
        double derivative = (error - lastError) / dt;

        // Integral term with anti-windup
        if (clampedTargetVelocity > 0) {
            integralSum += error * dt;
            // Clamp integral to prevent windup
            integralSum = Math.max(-0.5 / flyWheelKi, Math.min(0.5 / flyWheelKi, integralSum));
        } else {
            integralSum = 0.0; // Reset integral when not running
        }

        // Calculate PID output
        double pidOutput = (flyWheelKp * error) + (flyWheelKi * integralSum) + (flyWheelKd * derivative);

        // Calculate feedforward
        double feedforward = 0.0;
        if (clampedTargetVelocity > 0) {
            feedforward = flyWheelKs + (clampedTargetVelocity * flyWheelKf);
        }

        // Combine and apply voltage compensation
        double totalOutput = (pidOutput + feedforward) * getVoltageCompensation();

        // Clamp output to valid range
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        // Apply power to motors
        leftFlyWheelMotor.setPower(totalOutput);
        rightFlyWheelMotor.setPower(totalOutput);

        // Update state for next iteration
        lastError = error;
        timer.reset();

        // Update debug values
        debugCurrentVelocity = currentVelocity;
        debugTargetVelocity = clampedTargetVelocity;
        debugError = error;
        debugOutput = totalOutput;
        debugVoltage = currentVoltage;
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
                // Fall through to default
            }
        }
        return nominalVoltage; // Default if sensor unavailable
    }

    /**
     * Check if flywheels are at target velocity (within tolerance)
     */
    public boolean isAtTargetVelocity() {
        return Math.abs(debugCurrentVelocity - targetVelocity) < velocityTolerance;
    }

    /* ---------------- Commands ---------------- */

    /**
     * Command to spin up the flywheels to a specified velocity
     */
    public Command spinUp(double velocity) {
        return new InstantCommand(() -> {
            targetVelocity = velocity;
            isEnabled = true;
            integralSum = 0.0;
            lastError = 0.0;
            timer.reset();
        }).requires(this);
    }

    /**
     * Command to spin up flywheels to the default target velocity
     */
    public Command spinUp() {
        return spinUp(targetVelocity > 0 ? targetVelocity : maxVelocityAtNominalVoltage * 0.8);
    }

    /**
     * Command to stop the flywheels
     */
    public Command stop() {
        return new InstantCommand(() -> {
            isEnabled = false;
            targetVelocity = 0.0;
            integralSum = 0.0;
            if (leftFlyWheelMotor != null) leftFlyWheelMotor.setPower(0);
            if (rightFlyWheelMotor != null) rightFlyWheelMotor.setPower(0);
        }).requires(this);
    }

    /**
     * Command to toggle flywheel on/off
     */
    public Command toggle() {
        return new InstantCommand(() -> {
            if (isEnabled) {
                isEnabled = false;
                targetVelocity = 0.0;
                integralSum = 0.0;
                if (leftFlyWheelMotor != null) leftFlyWheelMotor.setPower(0);
                if (rightFlyWheelMotor != null) rightFlyWheelMotor.setPower(0);
            } else {
                targetVelocity = maxVelocityAtNominalVoltage * 0.8;
                isEnabled = true;
                integralSum = 0.0;
                lastError = 0.0;
                timer.reset();
            }
        }).requires(this);
    }

    /**
     * Command that spins up and waits for flywheels to reach speed
     * Uses a configurable delay time - adjust SPINUP_TIME_MS based on your shooter
     */
    public static long SPINUP_TIME_MS = 1500; // Time in ms to wait for spinup

    public Command spinUpAndWait(double velocity) {
        return new SequentialGroup(
            spinUp(velocity),
            new Delay(SPINUP_TIME_MS)
        );
    }

    /**
     * Command that spins up to default velocity and waits
     */
    public Command spinUpAndWait() {
        return spinUpAndWait(targetVelocity > 0 ? targetVelocity : maxVelocityAtNominalVoltage * 0.8);
    }

    /**
     * Set target velocity (can be called directly for fine control)
     */
    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        if (!isEnabled && velocity > 0) {
            isEnabled = true;
            integralSum = 0.0;
            lastError = 0.0;
            timer.reset();
        }
    }

    /**
     * Adjust target velocity by a delta amount
     */
    public void adjustTargetVelocity(double delta) {
        targetVelocity = Math.max(0, targetVelocity + delta);
    }

    /**
     * Stop the shooter directly (not a Command) - useful for emergency stops
     */
    public void stopShooter() {
        isEnabled = false;
        targetVelocity = 0.0;
        integralSum = 0.0;
        if (leftFlyWheelMotor != null) leftFlyWheelMotor.setPower(0);
        if (rightFlyWheelMotor != null) rightFlyWheelMotor.setPower(0);
    }

    /* ---------------- Getters ---------------- */

    public boolean isEnabled() {
        return isEnabled;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocityReading() {
        return debugCurrentVelocity;
    }

    public double getError() {
        return debugError;
    }
}







