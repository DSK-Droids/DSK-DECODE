package org.firstinspires.ftc.teamcode.DecodeNationals;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Flywheel Tuning OpMode
 *
 * Use this OpMode to tune the flywheel PID values.
 * Includes intake, transfer, and blocker controls for feeding balls.
 *
 * TUNING GUIDE:
 * 1. Start with all PID values at 0, set flyWheelKs = 0.05 and flyWheelKf = 0.0003
 * 2. Increase Kf until the flywheel reaches ~80% of target speed
 * 3. Increase Kp until the flywheel reaches target with minimal oscillation
 * 4. Add small Kd if there's oscillation
 * 5. Add small Ki if there's steady-state error
 *
 * Controls:
 * Gamepad 1:
 * - A: Toggle shooter on/off
 * - B: Stop shooter
 * - X: Emergency stop all
 * - Y: Reset target velocity to default
 * - Right Bumper: Increase target velocity (+100)
 * - Left Bumper: Decrease target velocity (-100)
 * - DPad Up: Large increase (+500)
 * - DPad Down: Large decrease (-500)
 * - Right Trigger: Run intake (hold)
 * - Left Trigger: Run transfer (hold)
 * - Right Stick Button: Toggle blocker
 */
@Configurable
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class FlywheelTuningOpMode extends NextFTCOpMode {

    // Default target velocity for tuning
    public static double DEFAULT_TARGET_VELOCITY = 2000.0;

    // Velocity adjustment amounts
    public static double SMALL_ADJUSTMENT = 100.0;
    public static double LARGE_ADJUSTMENT = 500.0;

    public FlywheelTuningOpMode() {
        addComponents(
                new SubsystemComponent(
                        ShooterSubsystem.INSTANCE,
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
        // Initialize subsystems
        ShooterSubsystem.INSTANCE.initialize(hardwareMap);
        IntakeSubsystem.INSTANCE.initialize(hardwareMap);
        TransferSubsystem.INSTANCE.initialize(hardwareMap);
        ShooterBlockerSubsystem.INSTANCE.initialize();

        // Set default target velocity
        ShooterSubsystem.INSTANCE.setTargetVelocity(DEFAULT_TARGET_VELOCITY);

        // --- Button Bindings ---

        // A: Toggle shooter
        Gamepads.gamepad1().a().whenBecomesTrue(ShooterSubsystem.INSTANCE.toggle());

        // B: Stop shooter
        Gamepads.gamepad1().b().whenBecomesTrue(ShooterSubsystem.INSTANCE.stop());

        // Y: Reset to default velocity
        Gamepads.gamepad1().y().whenBecomesTrue(
                new dev.nextftc.core.commands.utility.InstantCommand(() ->
                        ShooterSubsystem.INSTANCE.setTargetVelocity(DEFAULT_TARGET_VELOCITY))
        );

        // X: Emergency stop all
        Gamepads.gamepad1().x().whenBecomesTrue(
                new dev.nextftc.core.commands.utility.InstantCommand(() -> {
                    ShooterSubsystem.INSTANCE.stopShooter();
                    IntakeSubsystem.INSTANCE.stopIntake();
                    TransferSubsystem.INSTANCE.stopTransfer();
                    ShooterBlockerSubsystem.INSTANCE.block();
                })
        );

        // Right stick button: Toggle blocker
        Gamepads.gamepad1().rightStickButton().whenBecomesTrue(ShooterBlockerSubsystem.INSTANCE.toggle());
    }

    @Override
    public void onStartButtonPressed() {
        // Open blocker by default when starting
        ShooterBlockerSubsystem.INSTANCE.open();
    }

    @Override
    public void onUpdate() {
        // --- Trigger-based controls for intake/transfer ---

        // Right trigger: Run intake
        if (gamepad1.right_trigger > 0.1) {
            IntakeSubsystem.INSTANCE.runIntake();
        } else {
            IntakeSubsystem.INSTANCE.stopIntake();
        }

        // Left trigger: Run transfer
        if (gamepad1.left_trigger > 0.1) {
            TransferSubsystem.INSTANCE.runTransfer();
        } else {
            TransferSubsystem.INSTANCE.stopTransfer();
        }

        // --- Bumper-based velocity adjustments ---

        // Right bumper: Small increase
        if (gamepad1.right_bumper) {
            ShooterSubsystem.INSTANCE.adjustTargetVelocity(SMALL_ADJUSTMENT * 0.1); // Scale for per-loop
        }

        // Left bumper: Small decrease
        if (gamepad1.left_bumper) {
            ShooterSubsystem.INSTANCE.adjustTargetVelocity(-SMALL_ADJUSTMENT * 0.1);
        }

        // --- DPad-based large velocity adjustments ---

        // DPad Up: Large increase (only on press)
        if (gamepad1.dpad_up) {
            ShooterSubsystem.INSTANCE.adjustTargetVelocity(LARGE_ADJUSTMENT * 0.05);
        }

        // DPad Down: Large decrease (only on press)
        if (gamepad1.dpad_down) {
            ShooterSubsystem.INSTANCE.adjustTargetVelocity(-LARGE_ADJUSTMENT * 0.05);
        }

        // Update telemetry
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addLine("╔══════════════════════════════════╗");
        telemetry.addLine("║     FLYWHEEL TUNING OPMODE       ║");
        telemetry.addLine("╚══════════════════════════════════╝");

        telemetry.addLine("");
        telemetry.addLine("=== FLYWHEEL STATUS ===");
        telemetry.addData("State", ShooterSubsystem.INSTANCE.isEnabled() ? ">>> RUNNING <<<" : "STOPPED");
        telemetry.addData("Target Velocity", "%.0f ticks/sec", ShooterSubsystem.INSTANCE.getTargetVelocity());
        telemetry.addData("Current Velocity", "%.0f ticks/sec", ShooterSubsystem.INSTANCE.debugCurrentVelocity);
        telemetry.addData("Error", "%.0f ticks/sec", ShooterSubsystem.INSTANCE.debugError);
        telemetry.addData("At Target", ShooterSubsystem.INSTANCE.isAtTargetVelocity() ? "✓ YES" : "✗ NO");

        telemetry.addLine("");
        telemetry.addLine("=== PID OUTPUT ===");
        telemetry.addData("Motor Power", "%.4f", ShooterSubsystem.INSTANCE.debugOutput);
        telemetry.addData("Battery Voltage", "%.2fV", ShooterSubsystem.INSTANCE.debugVoltage);

        telemetry.addLine("");
        telemetry.addLine("=== CURRENT PID VALUES ===");
        telemetry.addData("Kp", "%.6f", ShooterSubsystem.flyWheelKp);
        telemetry.addData("Ki", "%.6f", ShooterSubsystem.flyWheelKi);
        telemetry.addData("Kd", "%.6f", ShooterSubsystem.flyWheelKd);
        telemetry.addData("Kf", "%.6f", ShooterSubsystem.flyWheelKf);
        telemetry.addData("Ks", "%.6f", ShooterSubsystem.flyWheelKs);

        telemetry.addLine("");
        telemetry.addLine("=== FEEDING SYSTEM ===");
        telemetry.addData("Intake", IntakeSubsystem.INSTANCE.isRunning() ? "RUNNING" : "stopped");
        telemetry.addData("Transfer", TransferSubsystem.INSTANCE.isRunning() ? "RUNNING" : "stopped");
        telemetry.addData("Blocker", ShooterBlockerSubsystem.INSTANCE.isBlocking() ? "BLOCKING" : "OPEN");

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A", "Toggle Shooter");
        telemetry.addData("B", "Stop Shooter");
        telemetry.addData("X", "Emergency Stop All");
        telemetry.addData("Y", "Reset Velocity to " + DEFAULT_TARGET_VELOCITY);
        telemetry.addData("RB/LB", "Adjust Velocity ±" + SMALL_ADJUSTMENT);
        telemetry.addData("DPad Up/Down", "Adjust Velocity ±" + LARGE_ADJUSTMENT);
        telemetry.addData("RT (hold)", "Run Intake");
        telemetry.addData("LT (hold)", "Run Transfer");
        telemetry.addData("R Stick Btn", "Toggle Blocker");

        telemetry.addLine("");
        telemetry.addLine("=== TUNING TIPS ===");
        telemetry.addLine("1. Use FTC Dashboard to adjust PID live");
        telemetry.addLine("2. Start: Kf=0.0003, Ks=0.05, others=0");
        telemetry.addLine("3. Increase Kf until ~80% speed");
        telemetry.addLine("4. Add Kp for remaining error");
        telemetry.addLine("5. Add Kd if oscillating");

        telemetry.update();
    }
}

