package org.firstinspires.ftc.teamcode.DecodeNationals;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * Shooter Blocker Subsystem for Nationals
 * Uses a Swyft Torque servo (320 deg rotation) to block/allow game pieces
 * from entering the shooter.
 * The servo rotates approximately 45 degrees between open and closed states.
 */
@Configurable
public class ShooterBlockerSubsystem implements Subsystem {

    public static final ShooterBlockerSubsystem INSTANCE = new ShooterBlockerSubsystem();

    private ShooterBlockerSubsystem() {}

    /* ---------------- Hardware ---------------- */
    private final ServoEx blockerServo = new ServoEx("blockerServo");

    /* ---------------- Configuration ---------------- */

    // Servo total range in degrees (Swyft Torque)
    public static double SERVO_RANGE_DEGREES = 320.0;

    // Rotation amount between states in degrees
    public static double ROTATION_DEGREES = 60;

    // Calculate servo position change for 45 degrees
    // 45 degrees / 320 degrees = ~0.14 servo position units
    public static double POSITION_CHANGE = ROTATION_DEGREES / SERVO_RANGE_DEGREES;

    // Servo positions for each state
    // Adjust these based on your mechanical setup
    public static double BLOCKED_POSITION = 0.3;  // Blocker is blocking (closed)
    public static double OPEN_POSITION = BLOCKED_POSITION + POSITION_CHANGE; // Blocker allows passage (open)

    /* ---------------- State ---------------- */
    private boolean isBlocking = true; // Start in blocking position
    private double currentPosition = BLOCKED_POSITION;

    // Debug/telemetry values
    public double debugServoPosition = BLOCKED_POSITION;
    public boolean debugIsBlocking = true;

    /* ---------------- Initialization ---------------- */

    /**
     * Initialize the blocker subsystem - call in onInit()
     */
    public void initialize() {
        // Start in blocking position
        isBlocking = true;
        currentPosition = BLOCKED_POSITION;
        blockerServo.setPosition(currentPosition);
    }

    /* ---------------- Periodic Update ---------------- */

    @Override
    public void periodic() {
        // Update debug values
        debugServoPosition = currentPosition;
        debugIsBlocking = isBlocking;
    }

    /* ---------------- Core Methods ---------------- */

    /**
     * Set the blocker to blocking position (closed)
     */
    public void block() {
        isBlocking = true;
        currentPosition = BLOCKED_POSITION;
        blockerServo.setPosition(currentPosition);
    }

    /**
     * Set the blocker to open position (allows game pieces through)
     */
    public void open() {
        isBlocking = false;
        currentPosition = OPEN_POSITION;
        blockerServo.setPosition(currentPosition);
    }

    /**
     * Toggle between blocking and open states
     */
    public void toggleState() {
        if (isBlocking) {
            open();
        } else {
            block();
        }
    }

    /**
     * Set servo position directly (for tuning)
     */
    public void setPosition(double position) {
        currentPosition = Math.max(0.0, Math.min(1.0, position));
        blockerServo.setPosition(currentPosition);
    }

    /* ---------------- Commands ---------------- */

    /**
     * Command to set blocker to blocking position
     */
    public Command blockCommand() {
        return new InstantCommand(this::block).requires(this);
    }

    /**
     * Command to set blocker to open position
     */
    public Command openCommand() {
        return new InstantCommand(this::open).requires(this);
    }

    /**
     * Command to toggle between states
     */
    public Command toggle() {
        return new InstantCommand(this::toggleState).requires(this);
    }

    /* ---------------- Getters ---------------- */

    public boolean isBlocking() {
        return isBlocking;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }
}

