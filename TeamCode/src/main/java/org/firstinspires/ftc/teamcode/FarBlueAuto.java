/*package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Example Auto", group = "Examples")
public class FarBlueAuto extends NextFTCOpMode {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public Command Path1Command ;
    public Command Path2Command;
    public Command Path3Command;
    public Command Path4Command ;
    public Command Path5Command ;

    public FarBlueAuto() {
        addComponents(
                new SubsystemComponent(
                        Flywheels.INSTANCE,
                        Intake.INSTANCE,
                        TurretSubsystem.INSTANCE
                ),

                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );


        Timer pathTimer, actionTimer, opmodeTimer;

         int pathState;

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(62.72700729927008, 9.261313868613128, -90));
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(62.727, 9.261),

                                new Pose(60.300, 24.400)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(300))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.300, 24.400),
                                new Pose(54.339, 35.794),
                                new Pose(40.000, 35.500)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(40.000, 35.500),

                                new Pose(9.000, 35.500)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 35.500),

                                new Pose(60.300, 24.400)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(300))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.300, 24.400),

                                new Pose(45.364, 43.323)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(0))

                .build();
        Path1Command = new FollowPath(Path1);
        Path2Command = new FollowPath(Path2);
        Path3Command = new FollowPath(Path3);
        Path4Command = new FollowPath(Path4);
        Path5Command = new FollowPath(Path5);
    }



    Command Shoot =
            new SequentialGroup(
                    Flywheels.INSTANCE.spin(),
                    new Delay(3000),
                    Transfer.INSTANCE.on
            );





    Command Auto = new SequentialGroup(
            Path1Command,
            Shoot,
            Path2Command,
            Path4Command,
            Path5Command
    );

    @Override
    public void onUpdate() {

        // These loop the movements of the robot, these must be called continuously in order to work
        Auto.schedule();


    }

    @Override
    public void onStartButtonPressed() {

    }
}*/