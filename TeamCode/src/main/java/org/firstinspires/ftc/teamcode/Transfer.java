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
public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();

    private Transfer() {
    }

    public boolean isStopped;
    private final MotorEx motor = new MotorEx("transfer").brakeMode();


    public final Command off= new InstantCommand(() -> { isStopped = true;});

    public final Command on = new InstantCommand(()-> { isStopped = false;});
    @Override
    public void periodic() {
        if (!isStopped) {
            motor.setPower(-1);
        } else {
            motor.setPower(0);
        }
    }

}