package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class blocker2 implements Subsystem {
    public static final blocker2 INSTANCE = new blocker2();
    private blocker2() { }

    private ServoEx servo2 = new ServoEx("blocker2");


    public Command block = new SetPosition(servo2, 0).requires(this);
    public Command unblock = new SetPosition(servo2, 1).requires(this);
}