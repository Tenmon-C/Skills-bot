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
    int test = 0;
    int LWheelCorrection;
    int RWheelCorrection;
    static final double COUNTS_PER_MOTOR_REV = 720;
    static final double DRIVE_GEAR_REDUCTION = 1;
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

        telemetry.addData("Step 1","Driving Forward");
        telemetry.update();
        drive(33.8,0.2);
        TelemeryClear();

        telemetry.addData("Step 2","Turning Right");
        telemetry.update();
        turnTo(-90,0.1);
        TelemeryClear();

        telemetry.addData("Step 3","Driving Forward");
        telemetry.update();
        drive(25,0.2);
        TelemeryClear();

        telemetry.addData("Step 4","Turning Right");
        telemetry.update();
        turnTo(-180,0.1);
        TelemeryClear();

        telemetry.addData("Step 5","Driving Forward");
        telemetry.update();
        drive(25,0.2);
        TelemeryClear();

        telemetry.addData("Step 6","Turning Right");
        telemetry.update();
        turnTo(-270,0.1);
        TelemeryClear();

        telemetry.addData("Step 7","Driving Forward");
        telemetry.update();
        drive(18,0.2);
        TelemeryClear();

        telemetry.addData("Step 8","Turning Left");
        telemetry.update();
        turnTo(-180,0.1);
        TelemeryClear();

        telemetry.addData("Step 7","Driving Forward");
        telemetry.update();
        drive(64,0.2);
        sleep(1000);
        ClawPos(.4);
        Arm.setTargetPosition(60);
        TelemeryClear();

        telemetry.addData("Step 8","Turning Left");
        telemetry.update();
        turnTo(-90,0.1);
        TelemeryClear();

        telemetry.addData("Step 9","Drive Forward");
        telemetry.update();
        drive(10,0.2);
        Arm.setTargetPosition(0);
        sleep(800);
        ClawPos(.55);
        TelemeryClear();

        telemetry.addData("Step 10","Driving Backwards");
        telemetry.update();
        drive(-10, 0.2);
        TelemeryClear();

        telemetry.addData("Step 11","Turning Left");
        telemetry.update();
        turnTo(0,0.1);
        TelemeryClear();

        telemetry.addData("Step 12","Driving Forward");
        telemetry.update();
        drive(64,0.2);
        TelemeryClear();
        ClawPos(0.4);
        sleep(800);
        Arm.setTargetPosition(30);

        telemetry.addData("Step 13","Turning Right");
        telemetry.update();
        turnTo(-180,0.1);
        Arm.setTargetPosition(80);
        TelemeryClear();

        telemetry.addData("Step 14","Driving Forward");
        telemetry.update();
        drive(64,0.2);
        TelemeryClear();

        telemetry.addData("Step 15","Turning Left");
        telemetry.update();
        turnTo(-90,0.1);
        TelemeryClear();

        telemetry.addData("Step 16", "Driving forward");
        telemetry.update();
        drive(10,0.2);
        Arm.setTargetPosition(0);
        sleep(500);
        ClawPos(0.55);
        TelemeryClear();
        
        telemetry.addData("Step 17","Driving Backwords");
        telemetry.update();
        drive(-10,0.2);
        TelemeryClear();

        telemetry.addData("Step 18","Turning Left");
        telemetry.update();
        turnTo(-0,0.1);
        TelemeryClear();

        telemetry.addData("Step 19","Driving Forward");
        telemetry.update();
        drive(17.75,0.2);
        TelemeryClear();

        telemetry.addData("Step 20","Turning Right");
        telemetry.update();
        turnTo(-90,0.1);
        TelemeryClear();

        telemetry.addData("Step 21", "Driving Forward");
        telemetry.update();
        drive(7,0.2);
        TelemeryClear();
        ClawPos(0.4);
        sleep(500);
        Arm.setTargetPosition(30);

        telemetry.addData("Step 22", "Turning Left");
        telemetry.update();
        turnTo(90,0.1);
        TelemeryClear();

        telemetry.addData("Step 23","Driving Forward");
        telemetry.update();
        Arm.setTargetPosition(100);
        drive(17,0.2);
        TelemeryClear();
        ClawPos(0.55);

        telemetry.addData("Step 24","Driving Backwords");
        telemetry.update();
        drive(-10,0.2);
        Arm.setTargetPosition(0);
        TelemeryClear();

        telemetry.addData("Step 25","Turning Right");
        telemetry.update();
        turnTo(0,0.1);
        TelemeryClear();

        telemetry.addData("Step 26","Drving Forward");
        telemetry.update();
        drive(6,0.2);
        TelemeryClear();

        telemetry.addData("Step 27","Turning Right");
        telemetry.update();
        turnTo(-90,0.1);
        TelemeryClear();

        telemetry.addData("Step 28","Driving Forward");
        telemetry.update();
        drive(7,0.2);
        ClawPos(0.4);
        sleep(500);
        Arm.setTargetPosition(100);
        TelemeryClear();

        telemetry.addData("Step 29","Turning Left");
        telemetry.update();
        turnTo(90,0.1);
        TelemeryClear();

        telemetry.addData("Step 29","Driving Forward");
        telemetry.update();
        drive(17,0.2);
        TelemeryClear();
        ClawPos(0.55);

        telemetry.addData("Step 30","Drivng Backwards");
        telemetry.update();
        drive(-10,0.2);
        Arm.setTargetPosition(0);
        TelemeryClear();

        telemetry.addData("Step 31","Turning Right");
        telemetry.update();
        turnTo(0,0.1);
        TelemeryClear();

        telemetry.addData("Step 32","Driving Forward");
        telemetry.update();
        drive(6,0.2);
        TelemeryClear();

        telemetry.addData("Step 33","Turning Right");
        telemetry.update();
        turnTo(-90,0.1);
        TelemeryClear();

        telemetry.addData("Step 34","Driving Forward");
        telemetry.update();
        drive(7,0.2);
        ClawPos(0.4);
        sleep(500);
        Arm.setTargetPosition(100);
        TelemeryClear();

        telemetry.addData("Step 35","Driving Backwards");
        telemetry.update();
        drive(-10,0.2);
        TelemeryClear();

        telemetry.addData("Step 36","Turning Right");
        telemetry.update();
        turnTo(-180,0.1);
        TelemeryClear();

        telemetry.addData("Step 37","Driving Forwards");
        telemetry.update();
        drive(6,0.2);
        TelemeryClear();

        telemetry.addData("Step 38","Turning Right");
        telemetry.update();
        turnTo(-270,0.1);
        TelemeryClear();

        telemetry.addData("Step 39","Driving Forwards");
        telemetry.update();
        drive(10,0.2);
        TelemeryClear();
        ClawPos(0.55);

        telemetry.addData("Step 40","Driving Backwards");
        telemetry.update();
        drive(10,0.2);
        TelemeryClear();
        Arm.setTargetPosition(0);

        telemetry.addData("Step 41","Turning Right");
        telemetry.update();
        turnTo(-0,0.1);
        TelemeryClear();

        telemetry.addData("Step 42","Driving Forwards");
        telemetry.update();
        drive(34.5,0.2);
        TelemeryClear();

        telemetry.addData("Step 43","Turning Right");
        telemetry.update();
        turnTo(-90,0.1);
        TelemeryClear();

        telemetry.addData("Step 44","Driving Forwards");
        telemetry.update();
        drive(17,0.2);
        TelemeryClear();

        telemetry.addData("Step 45","Turning Left");
        telemetry.update();
        turnTo(0,0.1);
        TelemeryClear();

        telemetry.addData("Step 46", "Signaling Ready For Pass-off");
        Arm.setTargetPosition(80);
        while(true){
            ClawPos(.4);
            sleep(500);
            ClawPos(.55);
            sleep(500);
            ++test;

            if(test > 5){
                break;
            }
        }
        TelemeryClear();

        Sensor(36);
    }

    public void turnTo(double targetAngle, double maxTurnSpeed) {
        if (!opModeIsActive()) return;

        // --- PID Constants ---
        final double Kp = 0.002;
        final double Kd = 0.015;
        final double HEADING_TOLERANCE = 1;

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


    public void drive(double distanceInches, double speed) {
        if (!opModeIsActive()) return;

        // --- HEADING CORRECTION SETUP ---
        // A proportional gain constant. This is the most important value to tune.
        // Start with a very small value (e.g., 0.01) and increase it until the robot corrects smoothly.
        // If it oscillates, the value is too high.
        final double P_DRIVE_GAIN = 0.01;

        // Read the starting angle from the IMU. This will be our target heading.
        double targetAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double turnCorrection;
        // --- END OF HEADING CORRECTION SETUP ---

        int targetTicks = (int)(distanceInches * COUNTS_PER_INCH);

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftWheel.setTargetPosition(targetTicks);
        RightWheel.setTargetPosition(targetTicks);

        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftWheel.setPower(speed);
        RightWheel.setPower(speed);

        // The main loop continues as long as the motors are busy.
        while (opModeIsActive() && (LeftWheel.isBusy() || RightWheel.isBusy())) {

            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            double error = targetAngle - currentAngle;

            // Handle the -180/180 degree wrap-around.
            if (error > 180)  error -= 360;
            if (error <= -180) error += 360;

            // Calculate the correction power using the proportional gain.
            turnCorrection = error * P_DRIVE_GAIN;

            double leftPower = speed - turnCorrection;
            double rightPower = speed + turnCorrection;

            // Clip the power values to ensure they are within the -1.0 to 1.0 range.
            leftPower = Range.clip(leftPower, -1.0, 1.0);
            rightPower = Range.clip(rightPower, -1.0, 1.0);

            // Set the adjusted power to the motors.
            LeftWheel.setPower(leftPower);
            RightWheel.setPower(rightPower);

            // Telemetry for debugging.
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Pos", "L: %7d, R: %7d",
                    LeftWheel.getCurrentPosition(),
                    RightWheel.getCurrentPosition());
            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Current Angle", "%.2f", currentAngle);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Correction", "%.3f", turnCorrection);
            telemetry.update();
        }

        LeftWheel.setPower(0);
        RightWheel.setPower(0);

        // Reset the motor modes to your default.
        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ClawPos(double Pos){
        RightClaw.setPosition(Pos);
        LeftClaw.setPosition(Pos);
}
    public void Sensor(int Distance) {
        int Check = 0;
        while(getDistance() < 40){
            sleep(1000);
            ++Check;
            if(Check == 10){
                break;
            }
        }

        if (!opModeIsActive() && getDistance() < 30 && Check == 10) return;
        double driveSpeed = 0.2;

        LeftWheel.setPower(driveSpeed);
        RightWheel.setPower(driveSpeed);
        while (opModeIsActive() && getDistance() > Distance) {
            telemetry.addData("Distance", getDistance());
            telemetry.update();
        }
        LeftWheel.setPower(0);
        RightWheel.setPower(0);
    }
    public void TelemeryClear(){
        telemetry.clear();

    }
}

/*        telemetry.addLine("Step 1: Driving forward");
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
        turnTo(180, 0.1);
        Arm.setTargetPosition(70);
        sleep(500);

        telemetry.addLine("Step 3: Driving to starting location");
        telemetry.update();
        drive(distanceTravelInches,0.2);
        ClawPos(0.55);

        telemetry.addLine("Autonomous Finished!");
        telemetry.update();
        sleep(2000);*/