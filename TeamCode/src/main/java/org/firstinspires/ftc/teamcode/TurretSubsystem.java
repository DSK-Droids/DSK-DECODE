package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
// We no longer need this import as the state is passed in
// import dev.nextftc.ftc.NextFTCOpMode;

public class TurretSubsystem implements Subsystem {

    // Singleton instance
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();

    // Hardware
    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private Telemetry telemetry;

    // --- TUNING VALUES ---
    public static double kP = 0.009;
    public static double kI = 0.0;
    public static double kD = 0.0006;
    public static double kS = 0.05;
    public static double DEADBAND = 1.0;

    // Encoder and Rotation Constants
    private static final double TICKS_PER_REVOLUTION = 145.6;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_REVOLUTION * GEAR_RATIO) / 360.0;
    private static final double MAX_ROTATION_DEGREES = 180.0;

    // State variables
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private boolean trackingEnabled = false;

    // Public command
    public Command toggleTrackingCommand;

    private TurretSubsystem() {}

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limelight.pipelineSwitch(0);
        limelight.start();
        toggleTrackingCommand = new InstantCommand(this::toggleTracking).requires(this);
    }

    // This empty override is needed for the interface now that we are manually calling the other periodic method.
    @Override
    public void periodic() {}

    /**
     * The main logic loop for the subsystem.
     * @param opModeIsActive The state of the parent OpMode, passed from its periodic() loop.
     */
    public void periodic(boolean opModeIsActive) {
        // FIX #1: Only run logic after the OpMode has started.
        if (!opModeIsActive) {
            if (turretMotor != null && turretMotor.getPower() != 0) {
                turretMotor.setPower(0.0);
            }
            return; // Don't do anything else until Start is pressed
        }

        // Prevent NullPointerException if initialize() hasn't been called
        if (turretMotor == null || limelight == null) return;

        if (!trackingEnabled) {
            if (turretMotor.getPower() != 0) turretMotor.setPower(0.0);
            return;
        }

        LLResult result = limelight.getLatestResult();
        double motorPower = 0.0;
        double currentDegrees = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        if (result.isValid() && !result.getFiducialResults().isEmpty()) {
            double error = result.getTx();

            if (Math.abs(currentDegrees) > MAX_ROTATION_DEGREES) {
                error += (currentDegrees > 0) ? -360.0 : 360.0;
            }

            if (Math.abs(error) > DEADBAND) {
                double dt = pidTimer.seconds();
                double derivative = 0;
                if (dt > 0) {
                    derivative = (error - lastError) / dt;
                }
                integralSum += error * dt;

                double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);
                motorPower = pidOutput + (kS * Math.signum(pidOutput));
            } else {
                motorPower = 0.0;
                integralSum = 0;
            }
            lastError = error;
        } else {
            motorPower = 0.2;
            lastError = 0;
            if (telemetry != null) telemetry.addData("Limelight", "Searching...");
        }

        turretMotor.setPower(Math.max(-1.0, Math.min(motorPower, 1.0)));
        pidTimer.reset();

        if (telemetry != null) {
            telemetry.addData("Turret Tracking", trackingEnabled ? "ACTIVE" : "INACTIVE");
            telemetry.addData("Turret Angle", "%.2f", currentDegrees);
            telemetry.addData("Target Error", "%.2f", lastError);
            telemetry.addData("Motor Power", "%.2f", motorPower);
        }
    }

    public void onDisable() {
        trackingEnabled = false;
        if(turretMotor != null) turretMotor.setPower(0.0);
        if (limelight != null) limelight.stop();
    }

    public void toggleTracking() {
        if(turretMotor == null) return;
        trackingEnabled = !trackingEnabled;
        if (trackingEnabled) {
            integralSum = 0;
            lastError = 0;
            pidTimer.reset();
        } else {
            turretMotor.setPower(0.0);
        }
    }

    public boolean isTracking() {
        return trackingEnabled;
    }
}
