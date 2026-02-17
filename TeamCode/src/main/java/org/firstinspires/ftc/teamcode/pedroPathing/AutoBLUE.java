package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Flywheels;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Transfer;
import org.firstinspires.ftc.teamcode.TurretSubsystem;
import org.firstinspires.ftc.teamcode.blocker;
import org.firstinspires.ftc.teamcode.newIntake;
import org.firstinspires.ftc.teamcode.newTransfer;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Auto Blue", group = "Examples")
public class AutoBLUE extends NextFTCOpMode {

    public AutoBLUE() {
        addComponents(
                new SubsystemComponent(
                        Flywheels.INSTANCE,
                        newIntake.INSTANCE,
                        TurretSubsystem.INSTANCE,
                        newTransfer.INSTANCE,
                        blocker.INSTANCE
                ),

                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE);
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(21, 123, Math.toRadians(145)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(61, 83, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(23, 87, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickupPrep1 = new Pose(46, 87, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupPrep2 = new Pose(48, 63, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(23, 63, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose controlPoint = new Pose(60, 80); // Middle (Second Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, pickPrep1, pickPrep2, grabPickup3, scorePickup3,fromPickup2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickPrep1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPrep1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPrep1.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPrep1, pickup1Pose))
                .setLinearHeadingInterpolation(pickupPrep1.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        pickPrep2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPrep2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPrep2.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPrep2, pickup2Pose))
                .setLinearHeadingInterpolation(pickupPrep2.getHeading(), pickup2Pose.getHeading())
                .build();



        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, scorePose, controlPoint))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to the backdrop/goal
                //follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // ARRIVED AT SCORE POSITION
                if (!follower.isBusy()) {
                    new SequentialGroup(
                            Flywheels.spin()


                    ).schedule();
                    follower.followPath(scorePreload);
                    setPathState(2);
                }
                break;

            case 2: // THE "WAIT FOR PIXELS" STATE
                // pathTimer resets when we enter case 2.
                // We need to stay here while the intake/transfer (which are now "on")
                // actually move the pixels. Give it 1.5 to 2 seconds.
                if (pathTimer.getElapsedTimeSeconds() > 8) {

                    if (!follower.isBusy()) {
                        new ParallelGroup(
                                newIntake.spin(),
                                newTransfer.spin()
                        ).schedule();

                        setPathState(3);
                    }
                }
                break;

            case 3: // ARRIVED AT SCORE POSITION

                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    if (!follower.isBusy()) {

                        new SequentialGroup(
                                newTransfer.stop(),
                                blocker.INSTANCE.unblock
                        ).schedule();

                        follower.followPath(pickPrep1);
                        setPathState(4);
                    }
                }

                break;

            case 4: // ARRIVED AT SCORE POSITION
                if (!follower.isBusy()) {

                    follower.followPath(grabPickup1);
                    setPathState(5);
                }
                break;

            case 5: // THE "WAIT FOR PIXELS" STATE
                // pathTimer resets when we enter case 2.
                // We need to stay here while the intake/transfer (which are now "on")
                // actually move the pixels. Give it 1.5 to 2 seconds.

                if (!follower.isBusy()) {
                    new ParallelGroup(
                            newIntake.stop(),
                            blocker.INSTANCE.block
                    ).schedule();
                    follower.followPath(scorePickup1);
                    setPathState(6);
                }

                break;

            case 6: // ARRIVED AT SCORE POSITION


                if (!follower.isBusy()) {

                    new SequentialGroup(
                            newIntake.spin(),
                            newTransfer.spin()
                    ).schedule();


                    setPathState(7);
                }


                break;

            case 7: // ARRIVED AT SCORE POSITION

                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    if (!follower.isBusy()) {

                        new SequentialGroup(
                                newTransfer.stop(),
                                blocker.INSTANCE.unblock
                        ).schedule();

                        follower.followPath(pickPrep2);
                        setPathState(8);
                    }
                }

                break;


            case 8: // THE "WAIT FOR PIXELS" STATE
                // pathTimer resets when we enter case 2.
                // We need to stay here while the intake/transfer (which are now "on")
                // actually move the pixels. Give it 1.5 to 2 seconds.

                if (!follower.isBusy()) {

                    follower.followPath(grabPickup2);
                    setPathState(9);
                }

                break;


            case 9: // THE "WAIT FOR PIXELS" STATE
                // pathTimer resets when we enter case 2.
                // We need to stay here while the intake/transfer (which are now "on")
                // actually move the pixels. Give it 1.5 to 2 seconds.

                if (!follower.isBusy()) {
                    new ParallelGroup(
                            newIntake.stop(),
                            blocker.INSTANCE.block
                    ).schedule();
                    follower.followPath(scorePickup2);
                    setPathState(10);
                }

                break;


            case 10: // ARRIVED AT SCORE POSITION

                if (!follower.isBusy()) {

                    new SequentialGroup(
                            newIntake.spin(),
                            newTransfer.spin()
                    ).schedule();


                    setPathState(11);


                }

                break;

        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void onUpdate() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void onWaitForStart() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void onStop() {
    }
}