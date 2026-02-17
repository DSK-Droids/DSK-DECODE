package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.builder.ControlSystemBuilder;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;




@Configurable
public class Flywheels implements Subsystem {

    public static final Flywheels INSTANCE = new Flywheels();

    private Flywheels() {
    }


    /* ---------------- Hardware ---------------- */

    public static final MotorEx f1 = new MotorEx("flywheel_motor");


    /* ---------------- Control ---------------- */

public static double P= 0.01;
    public static double D= 0.0;

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.01, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();



    /* ---------------- State ---------------- */

    public static double targetVelocity = 800;

    /**
     * To determine if we are spinning slow or at PID
     */
    public static boolean SpinUp = false;

    public static double motorRpm = 0.0;

    public double velocity;
    /* ---------------- Periodic ---------------- */

    @Override
    public void periodic() {

        velocity = f1.getVelocity();

        if (SpinUp) {
            controller.setGoal(
                    new KineticState(0.0, targetVelocity)
            );

            double power = controller.calculate(
                    new KineticState(0.0, Math.abs(f1.getVelocity()))
            );

            f1.setPower(power);


        } else {
            f1.setPower(0);

        }
// --- ADD TELEMETRY HERE ---
        // We use the NextFTC wrapper to access the active OpMode's telemetry

        // Note: Do not call telemetry.update() here; NextFTC handles that automatically!

    }

    /* ---------------- Commands ---------------- */

    /**
     * Spins the flywheel
     */
    public static Command spin() {
        return new InstantCommand(() -> SpinUp = true);
    }

    /**
     * Stops the flywheel to cruise speed
     */
    public static Command stop() {
        return new InstantCommand(() -> SpinUp = false);
    }

    /**
     * Hard stop
     */
    public static Command hardStop() {
        return new InstantCommand(() -> {
            SpinUp = false;
            ;
        });
    }



    /* ---------------- Utilities ---------------- */

    public static void updatePid(double velocity) {
        // targetVelocity = velocity;
    }

}
