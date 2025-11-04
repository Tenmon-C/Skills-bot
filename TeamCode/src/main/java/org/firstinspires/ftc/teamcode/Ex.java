package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime; // FIX #1: Added missing import for ElapsedTime
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Ex")
public class Ex extends Base {

    private BNO055IMU imu;
    boolean sensor = true;
    int LWheelCorrection;
    int RWheelCorrection;
    static final double COUNTS_PER_MOTOR_REV = 480;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initHardware();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "Imu");
        imu.initialize(parameters);

        waitForStart();
        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Step 1: Driving forward");
        telemetry.update();

        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Sensor(5);
        int distanceTravelTicks = LeftWheel.getCurrentPosition();
        double distanceTravelInches = distanceTravelTicks / COUNTS_PER_INCH;

        ClawPos(0.4);
        sleep(500);
        Arm.setTargetPosition(20);
        telemetry.addData("Traveled", "%.2f", distanceTravelInches);

        telemetry.addLine("Step 2: Turning backwards");
        telemetry.update();
        turnTo(180, 0.2);
        Arm.setTargetPosition(60);
        sleep(500);

        telemetry.addLine("Step 3: Driving to starting location");
        telemetry.update();
        driveForward(distanceTravelInches,0.2);
        ClawPos(0.55);

        telemetry.addLine("Autonomous Finished!");
        telemetry.update();
        sleep(2000);
    }

    public void turnTo(double targetAngle, double maxTurnSpeed) {
        if (!opModeIsActive()) return;

        // --- PID Constants ---
        final double Kp = 0.002;
        final double Kd = 0.015;
        final double HEADING_TOLERANCE = 1.5;

        double lastError = 0;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double error = targetAngle - currentAngle;

            if (error > 180)  error -= 360;
            if (error <= -180) error += 360;

            if (Math.abs(error) <= HEADING_TOLERANCE) {
                break;
            }

            double timerValue = timer.milliseconds();
            if (timerValue == 0) timerValue = 1; // Prevent division by zero on the first loop
            double errorChange = (error - lastError) / timerValue;
            timer.seconds();
            timer.reset(); //Resets the timer
            lastError = error; // Updates error

            double p_effect = Kp * error;
            double d_effect = Kd * errorChange;
            double turnPower = p_effect + d_effect;

            turnPower = Range.clip(turnPower, -maxTurnSpeed, maxTurnSpeed);

            LeftWheel.setPower(-turnPower);
            RightWheel.setPower(turnPower);

            telemetry.addData("Target", "%.2f", targetAngle);
            telemetry.addData("Current", "%.2f", currentAngle);
            telemetry.update();
        }

        LeftWheel.setPower(0);
        RightWheel.setPower(0);
    }

    public void driveForward(double distanceInches, double speed) {
        if (!opModeIsActive()) return;

        int targetTicks = (int)(distanceInches * COUNTS_PER_INCH);

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftWheel.setTargetPosition(targetTicks);
        RightWheel.setTargetPosition(targetTicks);

        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftWheel.setPower(speed);
        RightWheel.setPower(speed);

        while (opModeIsActive() && (LeftWheel.isBusy() || RightWheel.isBusy())) {
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Pos", "L: %7d, R: %7d",
                    LeftWheel.getCurrentPosition(),
                    RightWheel.getCurrentPosition());
            telemetry.update();
        }

        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
public void ClawPos(double Pos){
        RightClaw.setPosition(Pos);
        LeftClaw.setPosition(Pos);
}
    public void Sensor(int Distance) {
        if (!opModeIsActive()) return;
        double driveSpeed = 0.2; // Use a consistent, slow speed

        LeftWheel.setPower(driveSpeed);
        RightWheel.setPower(driveSpeed);
        // Loop until the sensor condition is met
        while (opModeIsActive() && getDistance() > Distance) {
            telemetry.addData("Distance:", getDistance());
            telemetry.update();
        }
        LeftWheel.setPower(0);
        RightWheel.setPower(0);
    }

}

