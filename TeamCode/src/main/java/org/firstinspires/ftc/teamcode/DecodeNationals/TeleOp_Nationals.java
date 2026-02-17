package org.firstinspires.ftc.teamcode.DecodeNationals;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * TeleOp for Nationals - Turret Aiming Test
 *
 * This OpMode tests the odometry-based turret aiming system.
 * The turret will automatically aim at a fixed target position on the field
 * as the robot moves around.
 *
 * Controls:
 * - Left Stick: Drive (forward/back, strafe)
 * - Right Stick X: Rotate
 * - A: Toggle auto-aiming
 * - B: Center turret
 * - Right Stick (when not auto-aiming): Manual turret control
 * - DPad Up/Down: Adjust target Y coordinate
 * - DPad Left/Right: Adjust target X coordinate
 */
@Configurable
@TeleOp(name = "TeleOp Nationals - Turret Test", group = "Nationals")
public class TeleOp_Nationals extends NextFTCOpMode {

    private Follower follower;
    private TelemetryManager telemetryM;

    // Starting pose - adjust based on where you place the robot
    public static double START_X = 45.0;
    public static double START_Y = 98.0;
    public static double START_HEADING = 225.0; // degrees

    public TeleOp_Nationals() {
        addComponents(
                new SubsystemComponent(
                        TurretAimSubsystem.INSTANCE
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

        // Initialize turret with follower reference
        TurretAimSubsystem.INSTANCE.initialize(follower);

        // Initialize telemetry
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // --- Button Bindings ---

        // A button: Toggle auto-aiming
        Gamepads.gamepad1().a().whenBecomesTrue(TurretAimSubsystem.INSTANCE.toggleAiming());

        // B button: Center the turret
        Gamepads.gamepad1().b().whenBecomesTrue(TurretAimSubsystem.INSTANCE.centerTurret());
    }

    @Override
    public void onStartButtonPressed() {
        // Start teleop drive mode with brake mode
        follower.startTeleopDrive();
    }

    @Override
    public void onUpdate() {
        // Update follower (odometry)
        follower.update();

        // Drive control - robot centric
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,   // Forward/back
                -gamepad1.left_stick_x,   // Strafe
                -gamepad1.right_stick_x,  // Rotate
                true                       // Robot centric
        );

        // Manual turret control when auto-aiming is disabled
        if (!TurretAimSubsystem.INSTANCE.isAimingEnabled()) {
            // Use right stick for manual turret control when not driving rotation
            if (Math.abs(gamepad1.right_stick_x) < 0.1) {
                TurretAimSubsystem.INSTANCE.manualAdjust(gamepad2.right_stick_x);
            }
        }

        // DPad to adjust target position for testing
        if (gamepad1.dpad_up) {
            TurretAimSubsystem.TARGET_Y += 0.5;
        }
        if (gamepad1.dpad_down) {
            TurretAimSubsystem.TARGET_Y -= 0.5;
        }
        if (gamepad1.dpad_right) {
            TurretAimSubsystem.TARGET_X += 0.5;
        }
        if (gamepad1.dpad_left) {
            TurretAimSubsystem.TARGET_X -= 0.5;
        }

        // Update telemetry
        updateTelemetry();
        telemetryM.update();
    }

    private void updateTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addLine("=== ROBOT POSITION ===");
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Heading", "%.2f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("");
        telemetry.addLine("=== TARGET ===");
        telemetry.addData("Target X", "%.2f", TurretAimSubsystem.TARGET_X);
        telemetry.addData("Target Y", "%.2f", TurretAimSubsystem.TARGET_Y);
        telemetry.addData("Distance", "%.2f in", TurretAimSubsystem.INSTANCE.getDistanceToTarget());

        telemetry.addLine("");
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Auto-Aim", TurretAimSubsystem.INSTANCE.isAimingEnabled() ? "ENABLED" : "DISABLED");
        telemetry.addData("Target Angle", "%.2f°", TurretAimSubsystem.INSTANCE.debugTargetAngle);
        telemetry.addData("Current Angle", "%.2f°", TurretAimSubsystem.INSTANCE.debugTurretAngle);
        telemetry.addData("Error", "%.2f°", TurretAimSubsystem.INSTANCE.getAngleError());
        telemetry.addData("Servo Position", "%.3f", TurretAimSubsystem.INSTANCE.debugServoPos);

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A", "Toggle Auto-Aim");
        telemetry.addData("B", "Center Turret");
        telemetry.addData("DPad", "Adjust Target Position");

        // This is required to actually display the telemetry on the Driver Station
        telemetry.update();
    }
}
