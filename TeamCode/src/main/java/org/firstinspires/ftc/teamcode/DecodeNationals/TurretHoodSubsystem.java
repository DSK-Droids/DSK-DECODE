package org.firstinspires.ftc.teamcode.DecodeNationals;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * Turret Hood Subsystem for Nationals
 * Uses a single servo to adjust the hood/launch angle of the shooter.
 */
@Configurable
public class TurretHoodSubsystem implements Subsystem {

    public static final TurretHoodSubsystem INSTANCE = new TurretHoodSubsystem();

    private TurretHoodSubsystem() {}

    /* ---------------- Hardware ---------------- */
    private final ServoEx hoodServo = new ServoEx("hoodServo");

    /* ---------------- Configuration ---------------- */

    // Servo position limits (adjust based on your mechanical setup)
    public static double HOOD_MIN_POSITION = 0.0;   // Lowest angle (flattest shot)
    public static double HOOD_MAX_POSITION = 1.0;   // Highest angle (steepest shot)

    // Preset positions for common scenarios
    public static double HOOD_FLAT = 0.2;           // For close-range flat shots
    public static double HOOD_MEDIUM = 0.5;         // Medium range
    public static double HOOD_HIGH = 0.8;           // For long-range high arc shots

    // Angle mapping (for telemetry)
    public static double HOOD_MIN_ANGLE_DEGREES = 15.0;  // Angle at MIN_POSITION
    public static double HOOD_MAX_ANGLE_DEGREES = 60.0;  // Angle at MAX_POSITION

    // Manual adjustment speed
    public static double MANUAL_ADJUST_SPEED = 0.02;

    /* ---------------- State ---------------- */
    private double currentPosition = HOOD_MEDIUM;

    // Debug/telemetry values
    public double debugHoodPosition = HOOD_MEDIUM;
    public double debugHoodAngle = 0.0;

    /* ---------------- Initialization ---------------- */

    /**
     * Initialize the hood subsystem - call in onInit()
     */
    public void initialize() {
        currentPosition = HOOD_MEDIUM;
        setHoodPosition(currentPosition);
    }

    /* ---------------- Periodic Update ---------------- */

    @Override
    public void periodic() {
        // Update debug values
        debugHoodPosition = currentPosition;
        debugHoodAngle = positionToAngle(currentPosition);
    }

    /* ---------------- Core Methods ---------------- */

    /**
     * Set the hood servo position directly
     * @param position Servo position (0.0 to 1.0)
     */
    public void setHoodPosition(double position) {
        currentPosition = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, position));
        hoodServo.setPosition(currentPosition);
    }

    /**
     * Convert servo position to angle in degrees (for telemetry)
     */
    private double positionToAngle(double position) {
        double normalizedPos = (position - HOOD_MIN_POSITION) / (HOOD_MAX_POSITION - HOOD_MIN_POSITION);
        return HOOD_MIN_ANGLE_DEGREES + normalizedPos * (HOOD_MAX_ANGLE_DEGREES - HOOD_MIN_ANGLE_DEGREES);
    }

    /**
     * Convert angle in degrees to servo position
     */
    private double angleToPosition(double angleDegrees) {
        double normalizedAngle = (angleDegrees - HOOD_MIN_ANGLE_DEGREES) / (HOOD_MAX_ANGLE_DEGREES - HOOD_MIN_ANGLE_DEGREES);
        return HOOD_MIN_POSITION + normalizedAngle * (HOOD_MAX_POSITION - HOOD_MIN_POSITION);
    }

    /**
     * Manual adjustment of hood position
     * @param delta Amount to adjust (-1 to 1, will be scaled)
     */
    public void manualAdjust(double delta) {

        currentPosition += delta * MANUAL_ADJUST_SPEED;
        currentPosition = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, currentPosition));
        hoodServo.setPosition(currentPosition);
    }

    /* ---------------- Commands ---------------- */

    /**
     * Command to set hood to flat position (close range)
     */
    public Command setFlat() {
        return new InstantCommand(() -> setHoodPosition(HOOD_FLAT)).requires(this);
    }

    /**
     * Command to set hood to medium position
     */
    public Command setMedium() {
        return new InstantCommand(() -> setHoodPosition(HOOD_MEDIUM)).requires(this);
    }

    /**
     * Command to set hood to high position (long range)
     */
    public Command setHigh() {
        return new InstantCommand(() -> setHoodPosition(HOOD_HIGH)).requires(this);
    }

    /**
     * Command to set hood to a specific position
     */
    public Command setPosition(double position) {
        return new InstantCommand(() -> setHoodPosition(position)).requires(this);
    }

    /**
     * Command to set hood to a specific angle in degrees
     */
    public Command setAngle(double angleDegrees) {
        return new InstantCommand(() -> setHoodPosition(angleToPosition(angleDegrees))).requires(this);
    }

    /**
     * Command to cycle through preset positions: Flat -> Medium -> High -> Flat
     */
    public Command cyclePresets() {
        return new InstantCommand(() -> {
            if (Math.abs(currentPosition - HOOD_FLAT) < 0.05) {
                setHoodPosition(HOOD_MEDIUM);
            } else if (Math.abs(currentPosition - HOOD_MEDIUM) < 0.05) {
                setHoodPosition(HOOD_HIGH);
            } else {
                setHoodPosition(HOOD_FLAT);
            }
        }).requires(this);
    }

    /* ---------------- Getters ---------------- */

    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getCurrentAngle() {
        return debugHoodAngle;
    }
}

