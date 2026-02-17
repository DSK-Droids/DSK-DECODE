package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class blocker implements Subsystem {
    public static final blocker INSTANCE = new blocker();
    private blocker() { }

    private ServoEx servo = new ServoEx("blocker");


    public Command block = new SetPosition(servo, 0).requires(this);
    public Command unblock = new SetPosition(servo, 1).requires(this);
}