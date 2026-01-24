package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class TurretHood implements Subsystem {
    public static final TurretHood INSTANCE = new TurretHood();
    private TurretHood() { }

    private ServoEx servo = new ServoEx("hood");

    public Command down = new SetPosition(servo, 0.5).requires(this);
    public Command up = new SetPosition(servo, 1).requires(this);
}