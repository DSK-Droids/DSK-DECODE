package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


import dev.nextftc.extensions.pedro.PedroComponent;



public class Far12 {

    /* ---------------- Alliance ---------------- */

    /* ---------------- Poses ---------------- */

    public Pose startPoint = new Pose(72, 72);
    public Pose cP1 = new Pose(87.304, 38.208);
    public Pose row1 = new Pose(110.0, 35.5);
    public Pose row1End = new Pose(129.0, 35.5);
    public Pose shoot = new Pose(93.0, 18.0);
    public Pose hPIntake = new Pose(133.0, 10.5);
    public Pose row2Start = new Pose(88.0, 20.0);
    public Pose cP2 = new Pose(87.304, 60.394);
    public Pose row2End = new Pose(110.0, 59.5);
    public Pose rampIntake = new Pose(130.0, 63.0);

    public Pose row3 = new Pose(110.0, 83.5, 0.0);
    public Pose row3End = new Pose(129.0, 83.5, 0.0);

    public Pose park = new Pose(102.0, 13.0);

    public Pose cyclePose = new Pose(
            130.0,
            21.35,
            Math.toRadians(30.0)
    );

    /* ---------------- Paths ---------------- */

    public PathChain toRow1;
    public PathChain row1Intake;
    public PathChain shootRow1;

    /* ---------------- Constructor ---------------- */



    /* ---------------- Pose Utilities ---------------- */

  /*  private Pose flipPose(Pose p) {
        return new Pose(
                144.0 - p.getX(),
                p.getY(),
                p.getHeading()
        );
    }

    private void flipAll() {
        startPoint = flipPose(startPoint);
        cP1 = flipPose(cP1);
        row1 = flipPose(row1);
        row1End = flipPose(row1End);
        shoot = flipPose(shoot);
        hPIntake = flipPose(hPIntake);
        row2Start = flipPose(row2Start);
        cP2 = flipPose(cP2);
        row2End = flipPose(row2End);
        rampIntake = flipPose(rampIntake);
        row3 = flipPose(row3);
        row3End = flipPose(row3End);
        park = flipPose(park);
    }

    /* ---------------- Path Builder ---------------- */

    public void buildPaths() {



        // -------- To Row 1 --------
        toRow1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower.getPose(),
                                startPoint,
                                row1
                        )
                )
                .setLinearHeadingInterpolation(
                        follower.getHeading(),
                        Math.toRadians(0.0)
                )
                .build();

        // -------- Row 1 Intake --------
        row1Intake = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                row1End
                        )
                )
                .setConstantHeadingInterpolation(
                        Math.toRadians(0.0)
                )
                .build();

        // -------- Shoot Row 1 --------
        shootRow1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(
                        follower.getHeading(),
                        Math.toRadians(355.0)
                )
                .build();
    }
}
