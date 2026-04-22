package org.firstinspires.ftc.teamcode.DecodeNationals;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Shooter Lookup Table Manager
 *
 * Uses interpolated lookup tables to automatically set hood angle and flywheel speed
 * based on distance to target. Fill in the LUT data from testing to enable accurate
 * shooting from any distance.
 *
 * How to tune:
 * 1. Set the robot at a known distance from the target
 * 2. Manually adjust hood angle and flywheel speed until shots are accurate
 * 3. Record the distance, hood position, and flywheel velocity
 * 4. Add the data point to the appropriate LUT
 * 5. Repeat at different distances (recommend 5-10 data points across your range)
 *
 * The system will interpolate between your data points for distances in between.
 */
@Configurable
public class ShooterLUT implements Subsystem {

    public static final ShooterLUT INSTANCE = new ShooterLUT();

    private ShooterLUT() {}

    /* ---------------- Lookup Tables ---------------- */

    // Hood position LUT: distance (inches) -> servo position (0.0-1.0)
    private final InterpolatedLUT hoodLUT = new InterpolatedLUT();

    // Flywheel velocity LUT: distance (inches) -> velocity (ticks/sec)
    private final InterpolatedLUT flywheelLUT = new InterpolatedLUT();

    /* ---------------- Configuration ---------------- */

    // Enable/disable automatic adjustment
    public static boolean autoAdjustEnabled = false;

    // Minimum time between auto-adjustments (to prevent jitter)
    public static double UPDATE_INTERVAL_MS = 100.0;
    private long lastUpdateTime = 0;

    // Debug values for telemetry
    public double debugDistance = 0.0;
    public double debugLutHoodPosition = 0.0;
    public double debugLutFlywheelVelocity = 0.0;

    /* ---------------- Initialization ---------------- */

    /**
     * Initialize the LUTs with default/example data points.
     * REPLACE THESE VALUES with your actual tuned values from testing!
     */
    public void initialize() {
        // Clear any existing data
        hoodLUT.clear();
        flywheelLUT.clear();

        // ============================================================
        // HOOD POSITION LUT
        // Format: hoodLUT.addDataPoint(distance_inches, servo_position)
        // Servo position: 0.0 = flattest, 1.0 = steepest
        // ============================================================

        // Example data points - REPLACE WITH YOUR TUNED VALUES
        hoodLUT.addDataPoint(24.8+7, 0.8);   // 2 feet - very flat
        hoodLUT.addDataPoint(37.8+7, 0.8);   // 4 feet
        hoodLUT.addDataPoint(46.46+7, 0.27);   // 6 feet
        hoodLUT.addDataPoint(56+7, 0.276);   // 8 feet
        hoodLUT.addDataPoint(68.5+7, 0.41);  // 10 feet
        hoodLUT.addDataPoint(85.6+7, 0.316);  // 12 feet - steep

        // ============================================================
        // FLYWHEEL VELOCITY LUT
        // Format: flywheelLUT.addDataPoint(distance_inches, velocity_ticks_per_sec)
        // ============================================================

        // Example data points - REPLACE WITH YOUR TUNED VALUES
        flywheelLUT.addDataPoint(24.8+7, 1100);   // 2 feet - lower speed
        flywheelLUT.addDataPoint(37.8+7, 1250);   // 4 feet
        flywheelLUT.addDataPoint(46.46+7, 1250);   // 6 feet
        flywheelLUT.addDataPoint(56+7, 1250);   // 8 feet
        flywheelLUT.addDataPoint(68.5+7, 1300);  // 10 feet
        flywheelLUT.addDataPoint(85.6+7, 1350);  // 12 feet - higher speed
    }

    /* ---------------- Periodic Update ---------------- */

    @Override
    public void periodic() {
        if (!autoAdjustEnabled) return;

        // Rate limit updates
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime < UPDATE_INTERVAL_MS) return;
        lastUpdateTime = currentTime;

        // Get current distance from turret subsystem
        double distance = TurretAimSubsystem.INSTANCE.getDistanceToTarget();
        debugDistance = distance;

        // Look up values from LUTs
        double hoodPosition = getHoodPositionForDistance(distance);
        double flywheelVelocity = getFlywheelVelocityForDistance(distance);

        debugLutHoodPosition = hoodPosition;
        debugLutFlywheelVelocity = flywheelVelocity;

        // Apply values to subsystems
        TurretHoodSubsystem.INSTANCE.setHoodPosition(hoodPosition);
        ShooterSubsystem.INSTANCE.setTargetVelocity(flywheelVelocity);
    }

    /* ---------------- LUT Access Methods ---------------- */

    /**
     * Get the interpolated hood position for a given distance
     */
    public double getHoodPositionForDistance(double distance) {
        return hoodLUT.get(distance);
    }

    /**
     * Get the interpolated flywheel velocity for a given distance
     */
    public double getFlywheelVelocityForDistance(double distance) {
        return flywheelLUT.get(distance);
    }

    /* ---------------- LUT Management Methods ---------------- */

    /**
     * Add a hood data point
     * @param distance Distance in inches
     * @param position Servo position (0.0-1.0)
     */
    public void addHoodDataPoint(double distance, double position) {
        hoodLUT.addDataPoint(distance, position);
    }

    /**
     * Add a flywheel data point
     * @param distance Distance in inches
     * @param velocity Flywheel velocity in ticks/sec
     */
    public void addFlywheelDataPoint(double distance, double velocity) {
        flywheelLUT.addDataPoint(distance, velocity);
    }

    /**
     * Add both hood and flywheel data points at once
     * @param distance Distance in inches
     * @param hoodPosition Servo position (0.0-1.0)
     * @param flywheelVelocity Flywheel velocity in ticks/sec
     */
    public void addDataPoint(double distance, double hoodPosition, double flywheelVelocity) {
        hoodLUT.addDataPoint(distance, hoodPosition);
        flywheelLUT.addDataPoint(distance, flywheelVelocity);
    }

    /**
     * Clear all hood data points
     */
    public void clearHoodLUT() {
        hoodLUT.clear();
    }

    /**
     * Clear all flywheel data points
     */
    public void clearFlywheelLUT() {
        flywheelLUT.clear();
    }

    /**
     * Clear all data points from both LUTs
     */
    public void clearAllLUTs() {
        hoodLUT.clear();
        flywheelLUT.clear();
    }

    /* ---------------- Commands ---------------- */

    /**
     * Command to enable automatic LUT-based adjustment
     */
    public Command enableAutoAdjust() {
        return new InstantCommand(() -> autoAdjustEnabled = true).requires(this);
    }

    /**
     * Command to disable automatic LUT-based adjustment
     */
    public Command disableAutoAdjust() {
        return new InstantCommand(() -> autoAdjustEnabled = false).requires(this);
    }

    /**
     * Command to toggle automatic LUT-based adjustment
     */
    public Command toggleAutoAdjust() {
        return new InstantCommand(() -> autoAdjustEnabled = !autoAdjustEnabled).requires(this);
    }

    /**
     * Command to apply LUT values once (without enabling auto-adjust)
     * Useful for taking a single shot at current distance
     */
    public Command applyLUTOnce() {
        return new InstantCommand(() -> {
            double distance = TurretAimSubsystem.INSTANCE.getDistanceToTarget();
            debugDistance = distance;

            double hoodPosition = getHoodPositionForDistance(distance);
            double flywheelVelocity = getFlywheelVelocityForDistance(distance);

            debugLutHoodPosition = hoodPosition;
            debugLutFlywheelVelocity = flywheelVelocity;

            TurretHoodSubsystem.INSTANCE.setHoodPosition(hoodPosition);
            ShooterSubsystem.INSTANCE.setTargetVelocity(flywheelVelocity);
        }).requires(this);
    }

    /**
     * Command to apply LUT values and spin up shooter, waiting until at speed
     */
    public Command prepareToShoot() {
        return new SequentialGroup(
            applyLUTOnce(),
            ShooterSubsystem.INSTANCE.spinUp(),
            new Delay(ShooterSubsystem.SPINUP_TIME_MS)
        );
    }

    /**
     * Command to apply LUT for a specific distance (for autonomous)
     */
    public Command applyForDistance(double distance) {
        return new InstantCommand(() -> {
            debugDistance = distance;

            double hoodPosition = getHoodPositionForDistance(distance);
            double flywheelVelocity = getFlywheelVelocityForDistance(distance);

            debugLutHoodPosition = hoodPosition;
            debugLutFlywheelVelocity = flywheelVelocity;

            TurretHoodSubsystem.INSTANCE.setHoodPosition(hoodPosition);
            ShooterSubsystem.INSTANCE.setTargetVelocity(flywheelVelocity);
        }).requires(this);
    }

    /* ---------------- Getters ---------------- */

    public boolean isAutoAdjustEnabled() {
        return autoAdjustEnabled;
    }

    public InterpolatedLUT getHoodLUT() {
        return hoodLUT;
    }

    public InterpolatedLUT getFlywheelLUT() {
        return flywheelLUT;
    }

    public int getHoodDataPointCount() {
        return hoodLUT.size();
    }

    public int getFlywheelDataPointCount() {
        return flywheelLUT.size();
    }
}

