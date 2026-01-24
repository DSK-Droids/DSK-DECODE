package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Sensor: Limelight3A", group = "TeleOp")
public class limelightTest extends LinearOpMode {
    private Limelight3A limelight;
    private DcMotorEx turretMotor;

    // PID constants - Adjusted to reduce jerky motion and overshooting
    public static double kP = 0.03; // Restored to previous value
    public static double kI = 0.0;
    public static double kD = 0.0012; // Drastically reduced to prevent jerky over-correction, was 0.0009

    // Deadband to prevent jittering around the target
    public static double DEADBAND = 1.0; // In degrees

    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            double motorPower = 0.0;

            if (result.isValid() && !result.getFiducialResults().isEmpty()) {
                double error = result.getTx();

                // Check if the error is outside the deadband
                if (Math.abs(error) > DEADBAND) {
                    // PID calculations
                    double derivative = 0;
                    if (pidTimer.seconds() > 0) { // Guard against division by zero
                        derivative = (error - lastError) / pidTimer.seconds();
                    }
                    integralSum += error * pidTimer.seconds();

                    motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);
                } else {
                    // Inside deadband, stop the motor
                    integralSum = 0; // Reset integral sum to prevent buildup
                }

                turretMotor.setPower(motorPower);

                lastError = error;
                pidTimer.reset();

                telemetry.addData("tx", result.getTx());
                telemetry.addData("Target Error", error);
                telemetry.addData("Motor Power", motorPower);
            } else {
                turretMotor.setPower(0);
                telemetry.addData("Limelight", "Searching");
            }

            telemetry.update();
            // Add a small sleep to stabilize the loop frequency (e.g., 50Hz)
            sleep(20);
        }
        limelight.stop();
        turretMotor.setPower(0.0);
    }
}
