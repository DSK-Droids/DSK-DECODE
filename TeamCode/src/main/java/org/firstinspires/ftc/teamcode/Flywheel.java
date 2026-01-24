package org.firstinspires.ftc.teamcode;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

/**
 * The flywheel subsystem implemented from
 * <a href="https://v1.nextftc.dev/guide/subsystems/flywheel">docs</a>.
 */
public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();

    private Flywheel() {
    }

    public boolean isStopped;
    private final MotorEx motor = new MotorEx("flywheel_motor").floatMode();

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();


    public final Command off= new InstantCommand(() -> { isStopped = true;});

    public final Command on = new SequentialGroup(
            new InstantCommand(()-> { isStopped = false;}),
            new RunToVelocity(controller, 1200)
    );
    @Override
    public void periodic() {
        if (!isStopped) {
            motor.setPower(controller.calculate(motor.getState()));
        } else {
            motor.setPower(0);
        }
    }

}