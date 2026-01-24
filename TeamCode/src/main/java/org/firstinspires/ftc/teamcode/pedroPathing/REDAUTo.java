package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Flywheel;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Transfer;
import org.firstinspires.ftc.teamcode.TurretHood;
import org.firstinspires.ftc.teamcode.blocker;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "RED Auto", group = "RED")
public class REDAUTo extends OpMode {


    private void addComponents(SubsystemComponent subsystemComponent, BulkReadComponent instance, BindingsComponent instance1) {
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(122.5, 122.5, Math.toRadians(217.5)); // Start Pose of our robot.
    private final Pose shootPose = new Pose(96, 95.4, Math.toRadians(225)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose preparePickup = new Pose(100.5, 83.7, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose Pickup1 = new Pose(118.5, 84.3, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose preparePickup2 = new Pose(96.7, 59.6, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose Pickup2 = new Pose(118.7, 59.6, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path shootPre;
    private PathChain prepPick1, Pick1, shootArt1, prepPick2, Pick2, shootArt2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        shootPre = new Path(new BezierLine(startPose, shootPose));
        shootPre.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        prepPick1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preparePickup))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preparePickup.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Pick1 = follower.pathBuilder()
                .addPath(new BezierLine(preparePickup, Pickup1))
                .setLinearHeadingInterpolation(preparePickup.getHeading(), Pickup1.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        shootArt1 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup1, shootPose))
                .setLinearHeadingInterpolation(Pickup1.getHeading(), shootPose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        prepPick2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preparePickup2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preparePickup2.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Pick2 = follower.pathBuilder()
                .addPath(new BezierLine(preparePickup2, Pickup2))
                .setLinearHeadingInterpolation(preparePickup2.getHeading(), Pickup2.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        shootArt2 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup2, shootPose))
                .setLinearHeadingInterpolation(Pickup2.getHeading(), shootPose.getHeading())
                .build();
    }

        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    follower.followPath(shootPre);
                    setPathState(1);
                    break;
                case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if(!follower.isBusy()) {
                        /* Score Preload */
                        Flywheel.INSTANCE.on
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup1,true);
                        setPathState(2);
                    }
                    break;
                case 2:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                    if(!follower.isBusy()) {
                        /* Grab Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup1,true);
                        setPathState(3);
                    }
                    break;
                case 3:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if(!follower.isBusy()) {
                        /* Score Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup2,true);
                        setPathState(4);
                    }
                    break;
                case 4:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                    if(!follower.isBusy()) {
                        /* Grab Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup2,true);
                        setPathState(5);
                    }
                    break;
                case 5:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if(!follower.isBusy()) {
                        /* Score Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup3,true);
                        setPathState(6);
                    }
                    break;
                case 6:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                    if(!follower.isBusy()) {
                        /* Grab Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup3, true);
                        setPathState(7);
                    }
                    break;
                case 7:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if(!follower.isBusy()) {
                        /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                        setPathState(-1);
                    }
                    break;
            }
        }

/** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }