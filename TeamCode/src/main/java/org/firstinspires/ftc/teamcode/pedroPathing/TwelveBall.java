/*package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Far12;
import org.firstinspires.ftc.teamcode.Flywheel;
import org.firstinspires.ftc.teamcode.Flywheels;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Autonomous
@Configurable
public class TwelveBall extends NextFTCOpMode {

    private Timer timer;


    public Far12 far12;



    @Override
    public void onInit() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Transfer.INSTANCE, Flywheels.INSTANCE),

                BulkReadComponent.INSTANCE
        );




        follower.setStartingPose(new Pose(72, 72));
    }



    @Override
    public void onStartButtonPressed() {
        timer = new Timer();

        SequentialGroup autonomousSequence = new SequentialGroup(
                Flywheels.spin(),
                new Delay(1500), // 1.5.seconds


                // Intake from Row 2
                new FollowPath(far12.toRow1),
                new FollowPath(far12.row1Intake),

                // Hit ramp
                new TurnBy(Angle.fromRad(-Math.PI / 2)),

                // Shoot
                new FollowPath(far12.shootRow1)

        );

        autonomousSequence.schedule();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("follower X", follower.getPose().getX());
        telemetry.addData("follower Y", follower.getPose().getY());
        telemetry.addData("follower heading", follower.getHeading());
        telemetry.update();
    }
}
*/