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

    // Target position on the field (the goal/basket location)
    // These are in the same coordinate system as Pedro Pathing
    public static double TARGET_X = 8.0;  // Adjust to your target X coordinate
    public static double TARGET_Y = 126.0; // Adjust to your target Y coordinate

    // Turret offset from robot center (if turret is not at robot center)
    public static double TURRET_OFFSET_X = 0.0; // inches forward from robot center
    public static double TURRET_OFFSET_Y = 0.0; // inches left from robot center

    // Control parameters
    public static double DEADBAND_DEGREES = 2.0; // Don't move if within this range
    // Removed MAX_SERVO_SPEED - servos will now respond instantly

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
     * Core aiming logic - calculates angle to target and moves servos
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
        // The turret angle is relative to the robot's heading
        targetAngle = angleToTarget - robotHeading;

        // Normalize to -PI to PI
        targetAngle = normalizeAngle(targetAngle);

        // Convert to degrees for easier tuning
        double targetAngleDegrees = Math.toDegrees(targetAngle);

        // Update debug values
        debugTargetAngle = targetAngleDegrees;
        debugRobotHeading = Math.toDegrees(robotHeading);

        // Calculate current turret angle from servo position
        currentTurretAngle = (currentServoPosition - SERVO_CENTER) * SERVO_RANGE_DEGREES;
        debugTurretAngle = currentTurretAngle;

        // Calculate error
        angleError = targetAngleDegrees - currentTurretAngle;

        // Check if within deadband
        if (Math.abs(angleError) < DEADBAND_DEGREES) {
            return; // Already aimed, don't move
        }

        // Calculate new servo position - DIRECT, no smoothing for fastest response
        // Map the target angle to servo position
        double targetServoPosition = SERVO_CENTER + (targetAngleDegrees / SERVO_RANGE_DEGREES);

        // Clamp to valid servo range
        targetServoPosition = Math.max(0.0, Math.min(1.0, targetServoPosition));

        // Set position directly - let the servos move as fast as they can
        currentServoPosition = targetServoPosition;

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

    /**
     * Manual turret control - adds to current position
     * @param delta Amount to change servo position (-1 to 1 scale, will be scaled down)
     */
    public void manualAdjust(double delta) {
        if (aimingEnabled) return; // Don't allow manual when auto-aiming

        currentServoPosition += delta * 0.01; // Scale down for fine control
        currentServoPosition = Math.max(0.0, Math.min(1.0, currentServoPosition));
        setServoPositions(currentServoPosition);
    }

    /* ---------------- Getters ---------------- */

    public boolean isAimingEnabled() {
        return aimingEnabled;
    }

    public double getAngleError() {
        return angleError;
    }

    public double getCurrentServoPosition() {
        return currentServoPosition;
    }

    public double getDistanceToTarget() {
        return debugDistanceToTarget;
    }
}

