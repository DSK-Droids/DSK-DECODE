package org.firstinspires.ftc.teamcode.DecodeNationals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * Turret Aiming Subsystem for Nationals
 * Uses two Swyft Balance servos running in opposite directions on the same axis
 * for increased torque and speed. Aims at a target position using odometry.
 */
@Configurable
public class TurretAimSubsystem implements Subsystem {

    public static final TurretAimSubsystem INSTANCE = new TurretAimSubsystem();

    private TurretAimSubsystem() {}

    /* ---------------- Hardware ---------------- */
    // Two servos on the same axis, running opposite directions
    private final ServoEx turretServo1 = new ServoEx("turret1");
    private final ServoEx turretServo2 = new ServoEx("turret2");

    /* ---------------- Configuration ---------------- */

    // Servo range configuration (adjust based on your servo setup)
    // 0.5 = center position, 0.0 and 1.0 are the extremes
    public static double SERVO_CENTER = 0.5;
    public static double SERVO_RANGE_DEGREES = 320.0; // Total range of servo in degrees

    // Gear ratio: 1:1.3 - turret moves 1.3x for every 1x servo movement
    public static double GEAR_RATIO = 1.3;
    // Effective turret range in degrees (servo range * gear ratio)
    public static double TURRET_RANGE_DEGREES = SERVO_RANGE_DEGREES * GEAR_RATIO; // ~416 degrees

    // Target position on the field (the goal/basket location)
    // These are in the same coordinate system as Pedro Pathing
    public static double TARGET_X = 130;  // Adjust to your target X coordinate
    public static double TARGET_Y = 130; // Adjust to your target Y coordinate

    // Turret offset from robot center (if turret is not at robot center)
    public static double TURRET_OFFSET_X = 0.0; // inches forward from robot center
    public static double TURRET_OFFSET_Y = 0.0; // inches left from robot center

    // Control parameters
    public static double DEADBAND_DEGREES = 1; // Don't move if within this range

    // Speed limiting - applied to ALL movements to prevent gear slipping
    // Maximum servo position change per update cycle (0.0 to 1.0 scale)
    public static double MAX_SERVO_SPEED = 0.007; // Slow, gear-safe speed for all moves

    // Maximum turret rotation: 180 degrees total (90 each direction from center)
    public static double MAX_TURRET_ANGLE = 90.0; // degrees from center in each direction

    // Servo position limits derived from the 180° turret limit
    // 90 degrees of turret rotation = 90 / (320 * 1.3) ≈ 0.216 servo units from center
    public static double SERVO_MIN = SERVO_CENTER - (MAX_TURRET_ANGLE / (SERVO_RANGE_DEGREES * GEAR_RATIO));
    public static double SERVO_MAX = SERVO_CENTER + (MAX_TURRET_ANGLE / (SERVO_RANGE_DEGREES * GEAR_RATIO));

    /* ---------------- State ---------------- */

    private Follower follower;
    private boolean aimingEnabled = false;
    private double currentServoPosition = SERVO_CENTER;
    private double targetAngle = 0.0;
    private double currentTurretAngle = 0.0;
    private double angleError = 0.0;

    // For telemetry/debugging
    public double debugTargetAngle = 0.0;
    public double debugRobotHeading = 0.0;
    public double debugTurretAngle = 0.0;
    public double debugServoPos = 0.5;
    public double debugDistanceToTarget = 0.0;

    /* ---------------- Initialization ---------------- */

    /**
     * Must be called in onInit() to provide the follower reference
     */
    public void initialize(Follower follower) {
        this.follower = follower;
        // Center the turret on init
        currentServoPosition = SERVO_CENTER;
        setServoPositions(SERVO_CENTER);
    }

    /* ---------------- Periodic Update ---------------- */

    @Override
    public void periodic() {
        if (follower == null) return;

        if (aimingEnabled) {
            updateTurretAiming();
        }

        // Update debug values for telemetry
        debugServoPos = currentServoPosition;
    }

    /**
     * Core aiming logic - calculates angle to target and moves servos.
     * Uses shortest-path error relative to the current turret angle so that
     * the turret always returns the correct way after hitting a limit.
     */
    private void updateTurretAiming() {
        Pose robotPose = follower.getPose();

        // Calculate turret position in field coordinates
        double robotHeading = robotPose.getHeading();
        double turretFieldX = robotPose.getX() +
            TURRET_OFFSET_X * Math.cos(robotHeading) - TURRET_OFFSET_Y * Math.sin(robotHeading);
        double turretFieldY = robotPose.getY() +
            TURRET_OFFSET_X * Math.sin(robotHeading) + TURRET_OFFSET_Y * Math.cos(robotHeading);

        // Calculate angle from turret to target (in field coordinates)
        double deltaX = TARGET_X - turretFieldX;
        double deltaY = TARGET_Y - turretFieldY;
        double angleToTarget = Math.atan2(deltaY, deltaX); // Radians, field-relative

        // Calculate distance for telemetry
        debugDistanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Convert to turret-relative angle (how much the turret needs to rotate from robot heading)
        // This is the raw desired angle in radians, NOT yet normalized
        double desiredAngleRad = angleToTarget - robotHeading;

        // Update debug
        debugRobotHeading = Math.toDegrees(robotHeading);

        // Calculate current turret angle from servo position (accounting for gear ratio)
        currentTurretAngle = (currentServoPosition - SERVO_CENTER) * SERVO_RANGE_DEGREES * GEAR_RATIO;
        debugTurretAngle = currentTurretAngle;

        // Compute the shortest-path error:
        // Find the difference between desired angle and current turret angle,
        // then normalize THAT to ±180° so the turret always takes the shortest route.
        double currentTurretAngleRad = Math.toRadians(currentTurretAngle);
        double rawError = desiredAngleRad - currentTurretAngleRad;
        rawError = normalizeAngle(rawError); // shortest path ±PI

        angleError = Math.toDegrees(rawError);

        // The actual target angle is: where the turret is now + shortest error
        double targetAngleDegrees = currentTurretAngle + angleError;

        debugTargetAngle = targetAngleDegrees;

        // Clamp target angle to what the turret can physically reach
        targetAngleDegrees = Math.max(-MAX_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, targetAngleDegrees));

        // Recalculate error after clamping
        angleError = targetAngleDegrees - currentTurretAngle;

        // Check if within deadband
        if (Math.abs(angleError) < DEADBAND_DEGREES) {
            return; // Already aimed, don't move
        }

        // Calculate desired servo position (accounting for gear ratio)
        double targetServoPosition = SERVO_CENTER + (targetAngleDegrees / (SERVO_RANGE_DEGREES * GEAR_RATIO));

        // Clamp to safe servo range
        targetServoPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, targetServoPosition));

        // Always rate-limit servo movement to protect gears
        double servoMovement = targetServoPosition - currentServoPosition;

        if (Math.abs(servoMovement) > MAX_SERVO_SPEED) {
            // Limit step size
            currentServoPosition += Math.signum(servoMovement) * MAX_SERVO_SPEED;
        } else {
            // Close enough - snap to target
            currentServoPosition = targetServoPosition;
        }

        // Clamp final position
        currentServoPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, currentServoPosition));

        // Set servo positions
        setServoPositions(currentServoPosition);
    }

    /**
     * Sets both servos - servo2 runs opposite direction
     */
    private void setServoPositions(double position) {
        turretServo1.setPosition(position);
        turretServo2.setPosition(1.0 - position); // Opposite direction
    }

    /**
     * Normalize angle to -PI to PI range
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /* ---------------- Commands ---------------- */

    public Command enableAiming() {
        return new InstantCommand(() -> {
            aimingEnabled = true;
        }).requires(this);
    }

    public Command disableAiming() {
        return new InstantCommand(() -> {
            aimingEnabled = false;
        }).requires(this);
    }

    public Command toggleAiming() {
        return new InstantCommand(() -> {
            aimingEnabled = !aimingEnabled;
        }).requires(this);
    }

    public Command centerTurret() {
        return new InstantCommand(() -> {
            aimingEnabled = false;
            currentServoPosition = SERVO_CENTER;
            setServoPositions(SERVO_CENTER);
        }).requires(this);
    }

    /* ---------------- Direct Methods (for use in lambdas) ---------------- */

    /**
     * Enable or disable aiming directly (not a Command)
     */
    public void setAimingEnabled(boolean enabled) {
        aimingEnabled = enabled;
    }

    /**
     * Center turret directly (not a Command) - for use in lambdas
     */
    public void centerTurretDirect() {
        aimingEnabled = false;
        currentServoPosition = SERVO_CENTER;
        setServoPositions(SERVO_CENTER);
    }

    /**
     * Manual turret control - adds to current position
     * @param delta Amount to change servo position (-1 to 1 scale, will be scaled down)
     */
    public void manualAdjust(double delta) {
        if (aimingEnabled) return; // Don't allow manual when auto-aiming

        currentServoPosition += delta * 0.005; // Scale down for fine control
        currentServoPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, currentServoPosition));
        setServoPositions(currentServoPosition);
    }

    /* ---------------- Getters ---------------- */

    public boolean isAimingEnabled() {
        return aimingEnabled;
    }

    public double getAngleError() {
        return angleError;
    }

    /**
     * Returns true if the turret is aimed within the deadband of the target
     */
    public boolean isAimed() {
        return aimingEnabled && Math.abs(angleError) < DEADBAND_DEGREES;
    }

    public double getCurrentServoPosition() {
        return currentServoPosition;
    }

    public double getDistanceToTarget() {
        return debugDistanceToTarget;
    }
}

