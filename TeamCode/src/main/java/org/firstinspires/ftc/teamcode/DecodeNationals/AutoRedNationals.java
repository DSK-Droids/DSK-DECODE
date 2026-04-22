package org.firstinspires.ftc.teamcode.DecodeNationals;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Autonomous for Red Alliance - Nationals
 *
 * Mirrored version of AutoBlueNationals across the X axis (144 - x, PI - heading).
 * Tracking + LUT auto-adjust are enabled for the ENTIRE auto.
 * The turret always aims at the goal and flywheel/hood continuously adjust.
 */
@Autonomous(name = "Auto Red - Nationals", group = "Nationals")
public class AutoRedNationals extends NextFTCOpMode {

    public AutoRedNationals() {
        addComponents(
                new SubsystemComponent(
                        ShooterSubsystem.INSTANCE,
                        IntakeSubsystem.INSTANCE,
                        TransferSubsystem.INSTANCE,
                        ShooterBlockerSubsystem.INSTANCE,
                        TurretAimSubsystem.INSTANCE,
                        TurretHoodSubsystem.INSTANCE,
                        ShooterLUT.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    /* ---------------- Hardware & Timers ---------------- */
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    /* ---------------- Poses (mirrored from Blue: x -> 144-x, heading -> PI-heading) ---------------- */
    private final Pose startPose = new Pose(123, 123, Math.toRadians(35));       // (123, 123, 35°)
    private final Pose scorePose = new Pose(144 - 61, 83, Math.toRadians(180 - 135));         // (83, 83, 45°)
    private final Pose pickupPrep1 = new Pose(144 - 46, 87, Math.toRadians(180 - 180));       // (98, 87, 0°)
    private final Pose pickup1Pose = new Pose(144 - 23, 87, Math.toRadians(180 - 180));       // (121, 87, 0°)
    private final Pose pickupPrep2 = new Pose(144 - 48, 63, Math.toRadians(180 - 180));       // (96, 63, 0°)
    private final Pose pickup2Pose = new Pose(144 - 23, 63, Math.toRadians(180 - 180));       // (121, 63, 0°)
    private final Pose controlPoint = new Pose(144 - 60, 80);                                  // (84, 80)
    private final Pose endPose = new Pose(129, 92, Math.toRadians(180));

    /* ---------------- Paths ---------------- */
    private Path scorePreload;
    private PathChain pickPrep1Path, grabPickup1Path, scorePickup1Path;
    private PathChain pickPrep2Path, grabPickup2Path, scorePickup2Path;
    private PathChain endPath;

    /* ---------------- Timing constants (seconds) ---------------- */
    private static final double SHOOT_TIME = 1.0;
    private static final double SPINUP_TIME = 1.5;

    /* ================================================================
     *  PATH BUILDING
     * ================================================================ */

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pickPrep1Path = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPrep1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPrep1.getHeading())
                .build();

        grabPickup1Path = follower.pathBuilder()
                .addPath(new BezierLine(pickupPrep1, pickup1Pose))
                .setLinearHeadingInterpolation(pickupPrep1.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        pickPrep2Path = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPrep2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPrep2.getHeading())
                .build();

        grabPickup2Path = follower.pathBuilder()
                .addPath(new BezierLine(pickupPrep2, pickup2Pose))
                .setLinearHeadingInterpolation(pickupPrep2.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2Path = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, controlPoint, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        endPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    /* ================================================================
     *  STATE MACHINE
     *  Tracking + LUT are always on. Only intake/transfer/blocker change.
     * ================================================================ */

    private void autonomousPathUpdate() {
        switch (pathState) {

            // ===== PRELOAD SCORE =====

            case 0: // Drive to score position (tracking + LUT already running)
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // Wait to arrive at score
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2: // Wait for spinup + turret aimed, then open blocker
                if (pathTimer.getElapsedTimeSeconds() > SPINUP_TIME
                        && TurretAimSubsystem.INSTANCE.isAimed()) {
                    new InstantCommand(() -> ShooterBlockerSubsystem.INSTANCE.open()).schedule();
                    setPathState(3);
                }
                break;

            case 3: // Feed balls through open blocker
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    new InstantCommand(() -> {
                        IntakeSubsystem.INSTANCE.runIntake();
                        TransferSubsystem.INSTANCE.runTransfer();
                    }).schedule();
                    setPathState(4);
                }
                break;

            case 4: // Wait for shot, then head to pickup 1
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    new InstantCommand(() -> {
                        ShooterBlockerSubsystem.INSTANCE.block();
                        IntakeSubsystem.INSTANCE.stopIntake();
                        TransferSubsystem.INSTANCE.stopTransfer();
                    }).schedule();
                    follower.followPath(pickPrep1Path);
                    setPathState(5);
                }
                break;

            // ===== PICKUP 1 =====

            case 5: // Arrive at pickup prep 1 → drive to pickup with intake on
                if (!follower.isBusy()) {
                    new InstantCommand(() -> IntakeSubsystem.INSTANCE.runIntake()).schedule();
                    follower.followPath(grabPickup1Path);
                    setPathState(6);
                }
                break;

            case 6: // Arrive at pickup 1 → stop intake, head back to score
                if (!follower.isBusy()) {
                    new InstantCommand(() -> {
                        IntakeSubsystem.INSTANCE.stopIntake();
                        ShooterBlockerSubsystem.INSTANCE.block();
                    }).schedule();
                    follower.followPath(scorePickup1Path);
                    setPathState(7);
                }
                break;

            // ===== SCORE PICKUP 1 =====

            case 7: // Wait to arrive at score
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;

            case 8: // Wait for spinup + turret aimed, then open blocker
                if (pathTimer.getElapsedTimeSeconds() > SPINUP_TIME
                        && TurretAimSubsystem.INSTANCE.isAimed()) {
                    new InstantCommand(() -> ShooterBlockerSubsystem.INSTANCE.open()).schedule();
                    setPathState(9);
                }
                break;

            case 9: // Feed balls
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    new InstantCommand(() -> {
                        IntakeSubsystem.INSTANCE.runIntake();
                        TransferSubsystem.INSTANCE.runTransfer();
                    }).schedule();
                    setPathState(10);
                }
                break;

            case 10: // Wait for shot, then head to pickup 2
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    new InstantCommand(() -> {
                        ShooterBlockerSubsystem.INSTANCE.block();
                        IntakeSubsystem.INSTANCE.stopIntake();
                        TransferSubsystem.INSTANCE.stopTransfer();
                    }).schedule();
                    follower.followPath(pickPrep2Path);
                    setPathState(11);
                }
                break;

            // ===== PICKUP 2 =====

            case 11: // Arrive at pickup prep 2 → drive to pickup with intake on
                if (!follower.isBusy()) {
                    new InstantCommand(() -> IntakeSubsystem.INSTANCE.runIntake()).schedule();
                    follower.followPath(grabPickup2Path);
                    setPathState(12);
                }
                break;

            case 12: // Arrive at pickup 2 → stop intake, head back to score
                if (!follower.isBusy()) {
                    new InstantCommand(() -> {
                        IntakeSubsystem.INSTANCE.stopIntake();
                        ShooterBlockerSubsystem.INSTANCE.block();
                    }).schedule();
                    follower.followPath(scorePickup2Path);
                    setPathState(13);
                }
                break;

            // ===== SCORE PICKUP 2 =====

            case 13: // Wait to arrive at score
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;

            case 14: // Wait for spinup + turret aimed, then open blocker
                if (pathTimer.getElapsedTimeSeconds() > SPINUP_TIME
                        && TurretAimSubsystem.INSTANCE.isAimed()) {
                    new InstantCommand(() -> ShooterBlockerSubsystem.INSTANCE.open()).schedule();
                    setPathState(15);
                }
                break;

            case 15: // Feed balls
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    new InstantCommand(() -> {
                        IntakeSubsystem.INSTANCE.runIntake();
                        TransferSubsystem.INSTANCE.runTransfer();
                    }).schedule();
                    setPathState(16);
                }
                break;

            // ===== END =====

            case 16: // Wait for shot, then drive to end position
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    new InstantCommand(() -> {
                        ShooterBlockerSubsystem.INSTANCE.block();
                        IntakeSubsystem.INSTANCE.stopIntake();
                        TransferSubsystem.INSTANCE.stopTransfer();
                        ShooterSubsystem.INSTANCE.stopShooter();
                        TurretAimSubsystem.INSTANCE.setAimingEnabled(false);
                        ShooterLUT.autoAdjustEnabled = false;
                        TurretAimSubsystem.INSTANCE.centerTurretDirect();
                    }).schedule();
                    follower.followPath(endPath);
                    setPathState(17);
                }
                break;

            case 17: // Wait to arrive at end position, then done
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /* ================================================================
     *  STATE HELPER
     * ================================================================ */

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /* ================================================================
     *  OPMODE LIFECYCLE
     * ================================================================ */

    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize all subsystems
        TurretAimSubsystem.INSTANCE.initialize(follower);
        ShooterSubsystem.INSTANCE.initialize(hardwareMap);
        TurretHoodSubsystem.INSTANCE.initialize();
        ShooterLUT.INSTANCE.initialize();
        IntakeSubsystem.INSTANCE.initialize(hardwareMap);
        TransferSubsystem.INSTANCE.initialize(hardwareMap);
        ShooterBlockerSubsystem.INSTANCE.initialize();

        // Set red alliance goal coordinates (mirrored from blue: 144 - 130 = 14)
        TurretAimSubsystem.TARGET_X = 140;
        TurretAimSubsystem.TARGET_Y = 140;


        // Build all paths
        buildPaths();
    }

    @Override
    public void onWaitForStart() {
        telemetry.addLine("=== AUTO RED - NATIONALS ===");
        telemetry.addLine("Ready to go!");
        telemetry.addData("Goal", "(%.0f, %.0f)", TurretAimSubsystem.TARGET_X, TurretAimSubsystem.TARGET_Y);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();

        // Enable tracking + LUT for the ENTIRE auto — never turned off until done
        TurretAimSubsystem.INSTANCE.setAimingEnabled(true);
        ShooterLUT.autoAdjustEnabled = true;

        setPathState(0);
    }

    @Override
    public void onUpdate() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("State", pathState);
        telemetry.addData("Timer", "%.1f s", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret Aimed", TurretAimSubsystem.INSTANCE.isAimed() ? "YES" : "NO");
        telemetry.addData("Turret Angle", "%.1f°", TurretAimSubsystem.INSTANCE.debugTurretAngle);
        telemetry.addData("Angle Error", "%.1f°", TurretAimSubsystem.INSTANCE.getAngleError());
        telemetry.addData("Flywheel", "%.0f / %.0f",
                ShooterSubsystem.INSTANCE.debugCurrentVelocity,
                ShooterSubsystem.INSTANCE.getTargetVelocity());
        telemetry.addData("Blocker", ShooterBlockerSubsystem.INSTANCE.isBlocking() ? "BLOCKED" : "OPEN");
        telemetry.addData("Distance", "%.1f in", TurretAimSubsystem.INSTANCE.getDistanceToTarget());
        telemetry.update();
    }

    @Override
    public void onStop() {
        ShooterSubsystem.INSTANCE.stopShooter();
        IntakeSubsystem.INSTANCE.stopIntake();
        TransferSubsystem.INSTANCE.stopTransfer();
        ShooterBlockerSubsystem.INSTANCE.block();
        TurretAimSubsystem.INSTANCE.setAimingEnabled(false);
        ShooterLUT.autoAdjustEnabled = false;
    }
}

