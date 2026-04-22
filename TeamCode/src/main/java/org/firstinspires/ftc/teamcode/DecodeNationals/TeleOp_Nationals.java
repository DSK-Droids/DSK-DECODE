package org.firstinspires.ftc.teamcode.DecodeNationals;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * TeleOp for Nationals - All controls on Gamepad 1
 *
 * During INIT, use DPad Left/Right to select alliance color (Red/Blue).
 * The goal coordinates are mirrored automatically.
 *
 * Controls (after start):
 * - Left Stick: Drive (forward/back, strafe)
 * - Right Stick X: Rotate
 * - Left Bumper (hold): INTAKE — close blocker, spin intake + transfer
 * - Right Bumper (hold): SHOOT — open blocker, spin intake + transfer to feed shooter
 * - A: Toggle tracking (turret auto-aim) + interpolated LUTs (auto hood + flywheel speed)
 * - B: Center turret + disable tracking
 * - X: Toggle flywheel manually
 * - Y: Emergency stop all
 * - DPad Left/Right: Manual turret adjust (when tracking off)
 * - DPad Up/Down: Manual hood adjust (when LUT off)
 */
@Configurable
@TeleOp(name = "TeleOp Nationals", group = "Nationals")
public class TeleOp_Nationals extends NextFTCOpMode {

    private Follower follower;
    private TelemetryManager telemetryM;

    // Starting pose
    public static double START_X = 61;
    public static double START_Y = 83;
   public static double START_HEADING = 135; // degrees

    //private final Pose endPose = new Pose(129, 92, Math.toRadians(180));


    // Alliance selection


    private boolean isBlueAlliance = true; // Default to blue

    // Goal coordinates for each alliance (Pedro Pathing coordinate system)
    // Blue alliance goal
    public static double BLUE_GOAL_X = 14;
    public static double BLUE_GOAL_Y = 130;
    // Red alliance goal (mirrored across the field: 144 - 130 = 14)
    public static double RED_GOAL_X = 14;
    public static double RED_GOAL_Y = 130;

    // Tracks whether the combined tracking+LUT mode is on
    private boolean trackingActive = false;

    public TeleOp_Nationals() {
        addComponents(
                new SubsystemComponent(
                        TurretAimSubsystem.INSTANCE,
                        ShooterSubsystem.INSTANCE,
                        TurretHoodSubsystem.INSTANCE,
                        ShooterLUT.INSTANCE,
                        IntakeSubsystem.INSTANCE,
                        TransferSubsystem.INSTANCE,
                        ShooterBlockerSubsystem.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING)));
        follower.update();

        // Initialize subsystems
        TurretAimSubsystem.INSTANCE.initialize(follower);
        ShooterSubsystem.INSTANCE.initialize(hardwareMap);
        TurretHoodSubsystem.INSTANCE.initialize();
        ShooterLUT.INSTANCE.initialize();
        IntakeSubsystem.INSTANCE.initialize(hardwareMap);
        TransferSubsystem.INSTANCE.initialize(hardwareMap);
        ShooterBlockerSubsystem.INSTANCE.initialize();

        // Apply default alliance (blue)
        applyAllianceGoal();

        // Initialize telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // --- All Gamepad 1 Bindings ---

        // LEFT BUMPER (hold): INTAKE — close blocker, spin intake + transfer inward
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(new InstantCommand(() -> {
            ShooterBlockerSubsystem.INSTANCE.block();
            IntakeSubsystem.INSTANCE.runIntake();
            TransferSubsystem.INSTANCE.runTransfer();
        }));
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(new InstantCommand(() -> {
            IntakeSubsystem.INSTANCE.stopIntake();
            TransferSubsystem.INSTANCE.stopTransfer();
        }));

        // RIGHT BUMPER (hold): SHOOT — open blocker, spin intake + transfer to feed shooter
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(new InstantCommand(() -> {
            ShooterBlockerSubsystem.INSTANCE.open();
            IntakeSubsystem.INSTANCE.runIntake();
            TransferSubsystem.INSTANCE.runTransfer();
        }));
        Gamepads.gamepad1().rightBumper().whenBecomesFalse(new InstantCommand(() -> {
            IntakeSubsystem.INSTANCE.stopIntake();
            TransferSubsystem.INSTANCE.stopTransfer();
            ShooterBlockerSubsystem.INSTANCE.block();
        }));

        // A: Toggle tracking (turret auto-aim + LUT auto-adjust + flywheel)
        Gamepads.gamepad1().a().whenBecomesTrue(new InstantCommand(() -> {
            trackingActive = !trackingActive;
            if (trackingActive) {
                // Enable turret tracking
                TurretAimSubsystem.INSTANCE.setAimingEnabled(true);
                // Enable LUT auto-adjust (sets hood + flywheel from distance)
                ShooterLUT.autoAdjustEnabled = true;
                // Spin up flywheel (LUT will set the velocity)
                if (!ShooterSubsystem.INSTANCE.isEnabled()) {
                    ShooterSubsystem.INSTANCE.setTargetVelocity(
                            ShooterLUT.INSTANCE.getFlywheelVelocityForDistance(
                                    TurretAimSubsystem.INSTANCE.getDistanceToTarget()));
                }
            } else {
                // Disable everything
                TurretAimSubsystem.INSTANCE.setAimingEnabled(false);
                ShooterLUT.autoAdjustEnabled = false;
                ShooterSubsystem.INSTANCE.stopShooter();
            }
        }));

        // B: Center turret + disable tracking
        Gamepads.gamepad1().b().whenBecomesTrue(new InstantCommand(() -> {
            trackingActive = false;
            TurretAimSubsystem.INSTANCE.setAimingEnabled(false);
            ShooterLUT.autoAdjustEnabled = false;
            TurretAimSubsystem.INSTANCE.centerTurretDirect();
        }));

        // X: Toggle flywheel manually
        Gamepads.gamepad1().x().whenBecomesTrue(ShooterSubsystem.INSTANCE.toggle());

        // Y: Emergency stop all
        Gamepads.gamepad1().y().whenBecomesTrue(new InstantCommand(() -> {
            trackingActive = false;
            TurretAimSubsystem.INSTANCE.setAimingEnabled(false);
            ShooterLUT.autoAdjustEnabled = false;
            ShooterSubsystem.INSTANCE.stopShooter();
            IntakeSubsystem.INSTANCE.stopIntake();
            TransferSubsystem.INSTANCE.stopTransfer();
            ShooterBlockerSubsystem.INSTANCE.block();
        }));

        // --- Gamepad 2 Debug ---



        // A: Relocalize to field center (72, 72) with 90° heading
        Gamepads.gamepad2().b().whenBecomesTrue(new InstantCommand(() -> {
            follower.setStartingPose(new Pose(136, 9, Math.toRadians(90)));
        }));
    }

    /**
     * Called every loop during INIT phase (before start is pressed).
     * Use DPad Left/Right to select alliance color.
     */
    @Override
    public void onWaitForStart() {
        // Alliance selection with DPad during init
        if (gamepad1.dpad_left) {
            isBlueAlliance = true;
            applyAllianceGoal();
        }
        if (gamepad1.dpad_right) {
            isBlueAlliance = false;
            applyAllianceGoal();
        }

        // Show alliance selection on telemetry
        telemetry.addLine("╔══════════════════════════════════╗");
        telemetry.addLine("║       ALLIANCE SELECTION         ║");
        telemetry.addLine("╚══════════════════════════════════╝");
        telemetry.addLine("");
        if (isBlueAlliance) {
            telemetry.addLine(">>>  BLUE ALLIANCE  <<<");
        } else {
            telemetry.addLine(">>>  RED ALLIANCE  <<<");
        }
        telemetry.addLine("");
        telemetry.addData("DPad Left", "Select BLUE");
        telemetry.addData("DPad Right", "Select RED");
        telemetry.addLine("");
        telemetry.addData("Goal X", "%.1f", TurretAimSubsystem.TARGET_X);
        telemetry.addData("Goal Y", "%.1f", TurretAimSubsystem.TARGET_Y);
        telemetry.update();
    }

    /**
     * Sets the turret target coordinates based on the selected alliance
     */
    private void applyAllianceGoal() {
        if (isBlueAlliance) {
            TurretAimSubsystem.TARGET_X = BLUE_GOAL_X;
            TurretAimSubsystem.TARGET_Y = BLUE_GOAL_Y;
        } else {
            TurretAimSubsystem.TARGET_X = RED_GOAL_X;
            TurretAimSubsystem.TARGET_Y = RED_GOAL_Y;
        }
    }

    @Override
    public void onStartButtonPressed() {
        follower.startTeleopDrive();
    }

    @Override
    public void onUpdate() {
        // Update follower (odometry)
        follower.update();

        // Drive control
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,   // Forward/back
                -gamepad1.left_stick_x,   // Strafe
                -gamepad1.right_stick_x,  // Rotate
                true                       // Robot centric
        );

        // DPad Left/Right: Manual turret adjust (only when tracking is off)
        if (!trackingActive) {
            if (gamepad1.dpad_left) {
                TurretAimSubsystem.INSTANCE.manualAdjust(-1.0);
            }
            if (gamepad1.dpad_right) {
                TurretAimSubsystem.INSTANCE.manualAdjust(1.0);
            }
        }

        // DPad Up/Down: Manual hood adjust (only when LUT auto-adjust is off)
        if (!ShooterLUT.INSTANCE.isAutoAdjustEnabled()) {
            if (gamepad1.dpad_up) {
                TurretHoodSubsystem.INSTANCE.manualAdjust(1.0);
            }
            if (gamepad1.dpad_down) {
                TurretHoodSubsystem.INSTANCE.manualAdjust(-1.0);
            }
        }

        // Update telemetry
        updateTelemetry();
        telemetryM.update();
    }

    private void updateTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addLine("=== ROBOT ===");
        telemetry.addData("Pos", "%.1f, %.1f  H: %.1f°", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));

        telemetry.addLine("");
        telemetry.addLine("=== TRACKING ===");
        telemetry.addData("Tracking + LUT", trackingActive ? ">>> ON <<<" : "OFF");
        telemetry.addData("Distance", "%.1f in", TurretAimSubsystem.INSTANCE.getDistanceToTarget());
        telemetry.addData("Turret Angle", "%.1f°", TurretAimSubsystem.INSTANCE.debugTurretAngle);
        telemetry.addData("Angle Error", "%.1f°", TurretAimSubsystem.INSTANCE.getAngleError());

        telemetry.addLine("");
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Flywheel", ShooterSubsystem.INSTANCE.isEnabled() ? "RUNNING" : "STOPPED");
        telemetry.addData("Velocity", "%.0f / %.0f", ShooterSubsystem.INSTANCE.debugCurrentVelocity, ShooterSubsystem.INSTANCE.getTargetVelocity());
        telemetry.addData("At Speed", ShooterSubsystem.INSTANCE.isAtTargetVelocity() ? "✓" : "✗");
        telemetry.addData("Blocker", ShooterBlockerSubsystem.INSTANCE.isBlocking() ? "BLOCKED" : "OPEN");

        telemetry.addLine("");
        telemetry.addLine("=== HOOD ===");
        telemetry.addData("Position", "%.3f", TurretHoodSubsystem.INSTANCE.debugHoodPosition);
        telemetry.addData("Angle", "%.1f°", TurretHoodSubsystem.INSTANCE.debugHoodAngle);

        telemetry.addLine("");
        telemetry.addLine("=== LUT DEBUG ===");
        telemetry.addData("LUT Enabled", ShooterLUT.autoAdjustEnabled ? "YES" : "NO");
        telemetry.addData("LUT Distance", "%.1f in", ShooterLUT.INSTANCE.debugDistance);
        telemetry.addData("LUT Hood", "%.3f", ShooterLUT.INSTANCE.debugLutHoodPosition);
        telemetry.addData("LUT Flywheel", "%.0f", ShooterLUT.INSTANCE.debugLutFlywheelVelocity);
        telemetry.addData("Goal", "(%.0f, %.0f)", TurretAimSubsystem.TARGET_X, TurretAimSubsystem.TARGET_Y);

        telemetry.addLine("");
        telemetry.addLine("=== INTAKE & TRANSFER ===");
        telemetry.addData("Intake", "%.0f / %.0f", IntakeSubsystem.INSTANCE.debugCurrentVelocity, IntakeSubsystem.INSTANCE.debugTargetVelocity);
        telemetry.addData("Transfer", "%.0f / %.0f", TransferSubsystem.INSTANCE.debugCurrentVelocity, TransferSubsystem.INSTANCE.debugTargetVelocity);

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("LB (hold)", "Intake (close blocker + feed in)");
        telemetry.addData("RB (hold)", "Shoot (open blocker + feed)");
        telemetry.addData("A", "Toggle Tracking + LUT");
        telemetry.addData("B", "Center Turret");
        telemetry.addData("X", "Toggle Flywheel");
        telemetry.addData("Y", "EMERGENCY STOP");
        telemetry.addData("DPad L/R", "Manual Turret");
        telemetry.addData("DPad U/D", "Manual Hood");
        telemetry.addData("GP2 B", "Center Turret (debug)");
        telemetry.addData("GP2 A", "Relocalize to (72, 72, 90°)");

        telemetry.update();
    }
}
