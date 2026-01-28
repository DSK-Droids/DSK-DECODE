package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp
public class ExampleTeleOp extends NextFTCOpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    public ExampleTeleOp() {
        addComponents(
                new SubsystemComponent(
                        Flywheels.INSTANCE,
                        newIntake.INSTANCE,
                        TurretSubsystem.INSTANCE,
                        newTransfer.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(45,98, Math.toRadians(45)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        // --- DEFINE BINDINGS ONCE HERE ---

        Gamepads.gamepad2().a().whenBecomesTrue(
                new SequentialGroup(
                        // Pulse 1
                        new ParallelGroup(newIntake.INSTANCE.spin(), newTransfer.INSTANCE.spin())
                )
                // Do NOT put .schedule() here
        );


        Gamepads.gamepad2().b().whenBecomesTrue(new SequentialGroup(
                        // Pulse 1
                        new ParallelGroup(newIntake.INSTANCE.stop(), newTransfer.INSTANCE.stop())
                )
                // Do NOT put .schedule() here
        );

        // Flywheels
        Gamepads.gamepad2().leftBumper().whenBecomesTrue(Flywheels.INSTANCE.spin());
        Gamepads.gamepad2().rightBumper().whenBecomesTrue(Flywheels.INSTANCE.stop());

        // Transfer
        Gamepads.gamepad2().x().whenBecomesTrue(newTransfer.INSTANCE.spin());
        Gamepads.gamepad2().y().whenBecomesTrue(newTransfer.INSTANCE.stop());
    }



    @Override
    public void onStartButtonPressed() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }



    @Override
    public void onUpdate() {
        //Call this once per loop
        follower.update();
        telemetryM.update();


        if (!automatedDrive) {
                //Make the last parameter false for field-centric
                //In case the drivers want to use a "slowMode" you can scale the vectors

                //This is the normal version to use in the TeleOp
                if (!slowMode) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // Robot Centric
                );

                    //This is how it looks with slowMode on
                else follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        true // Robot Centric
                );
            }

            //Automated PathFollowing
            if (gamepad1.aWasPressed()) {
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }

            //Stop automated following if the follower is done
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }









            //Optional way to change slow mode strength
            if (gamepad2.yWasPressed()) {
                slowModeMultiplier -= 0.25;
            }

      /* Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Flywheel.INSTANCE.on);


        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Flywheel.INSTANCE.off);

        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(TurretHood.INSTANCE.down);

        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(TurretHood.INSTANCE.up);

        Gamepads.gamepad2().a()
                .whenBecomesTrue(Transfer.INSTANCE.on);

        Gamepads.gamepad2().b()
                .whenBecomesTrue(Transfer.INSTANCE.off);

        Gamepads.gamepad2().x()
                .whenBecomesTrue(Intake.spin());

        Gamepads.gamepad2().y()
                .whenBecomesTrue(Intake.off());

//        Gamepads.gamepad1().rightBumper()
//                .whenBecomesTrue(blocker.INSTANCE.unblock);
//
//        Gamepads.gamepad1().leftBumper()
//                .whenBecomesTrue(blocker.INSTANCE.block);*/


            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());
            telemetryM.debug("automatedDrive", automatedDrive);



    }
}


