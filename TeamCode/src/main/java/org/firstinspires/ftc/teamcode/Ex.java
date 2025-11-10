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
    static final double COUNTS_PER_MOTOR_REV = 1680;
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

        telemetry.addData("Step 1","Driving Forward");//Exits the starting zone
        telemetry.update();
        drive(33.8,0.3);
        TelemeryClear();

        telemetry.addData("Step 2","Turning Right");//Turns to face the furthest wall
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 3","Driving Forward");//Drives till it reaches the auto zone
        telemetry.update();
        drive(25,0.3);
        TelemeryClear();

        telemetry.addData("Step 4","Turning Right");//Turns to stare at the auto zone
        telemetry.update();
        turnTo(-180,0.2);
        TelemeryClear();

        telemetry.addData("Step 5","Driving Forward");//Enters the auto zone
        telemetry.update();
        drive(25,0.3);
        TelemeryClear();

        telemetry.addData("Step 6","Turning Right");//Turns to the wall that leads to the supply's
        telemetry.update();
        turnTo(-270,0.2);
        TelemeryClear();

        telemetry.addData("Step 7","Driving Forward");//drives into the long hallway
        telemetry.update();
        drive(18,0.3);
        TelemeryClear();

        telemetry.addData("Step 8","Turning Left");//Turns left to be starting at the farthest wall in the hallway
        telemetry.update();
        turnTo(-180,0.2);
        TelemeryClear();

        telemetry.addData("Step 7","Driving Forward");//Drives to and picks up the supply closest to the safe zone
        telemetry.update();
        drive(64,0.3);
        sleep(1000);
        ClawPos(.4);
        Arm.setTargetPosition(60);
        TelemeryClear();

        telemetry.addData("Step 8","Turning Left");//Turns to the safe zone
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 9","Drive Forward");//Drives to the safe zone and drops the supply when there
        telemetry.update();
        drive(10,0.3);
        Arm.setTargetPosition(0);
        sleep(800);
        ClawPos(.55);
        TelemeryClear();

        telemetry.addData("Step 10","Driving Backwards");//Drives backwards out of the safe zone to where it picked up the supply
        telemetry.update();
        drive(-10, 0.3);
        TelemeryClear();

        telemetry.addData("Step 11","Turning Left");//Turns to face the other supply across the hallway
        telemetry.update();
        turnTo(0,0.2);
        TelemeryClear();

        telemetry.addData("Step 12","Driving Forward");//drives to the supply and grabs the supply
        telemetry.update();
        drive(64,0.3);
        TelemeryClear();
        ClawPos(0.4);
        sleep(800);
        Arm.setTargetPosition(30);

        telemetry.addData("Step 13","Turning Right");//Turns to the other side of the hallway
        telemetry.update();
        turnTo(-180,0.2);
        Arm.setTargetPosition(80);
        TelemeryClear();

        telemetry.addData("Step 14","Driving Forward");//Drives to the end of the hallway
        telemetry.update();
        drive(64,0.3);
        TelemeryClear();

        telemetry.addData("Step 15","Turning Left");//Turns toward the safe zone
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 16", "Driving forward");//Drives into the save zone and drops the supply off
        telemetry.update();
        drive(10,0.3);
        Arm.setTargetPosition(0);
        sleep(500);
        ClawPos(0.55);
        TelemeryClear();
        
        telemetry.addData("Step 17","Driving Backwords");//Leaves the safe zone
        telemetry.update();
        drive(-10,0.3);
        TelemeryClear();

        telemetry.addData("Step 18","Turning Left");//Turns to face the other side of the hallway
        telemetry.update();
        turnTo(-0,0.2);
        TelemeryClear();

        telemetry.addData("Step 19","Driving Forward");//Drives to the first spot where the ECU fan could be
        telemetry.update();
        drive(17.75,0.3);
        TelemeryClear();

        telemetry.addData("Step 20","Turning Right");//Turns to face the first location = pos 1
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 21", "Driving Forward");//Drives and picks up the ECU fan in pos 1
        telemetry.update();
        drive(7,0.3);
        TelemeryClear();
        ClawPos(0.4);
        sleep(500);
        Arm.setTargetPosition(30);

        telemetry.addData("Step 22", "Turning Left");//Turns to face the ECU fan zone
        telemetry.update();
        turnTo(90,0.2);
        TelemeryClear();

        telemetry.addData("Step 23","Driving Forward");//Drives to the ECU fan zone and drops the fan off
        telemetry.update();
        Arm.setTargetPosition(100);
        drive(17,0.3);
        TelemeryClear();
        ClawPos(0.55);

        telemetry.addData("Step 24","Driving Backwords");//drives backwards to not nock off the ECU fan
        telemetry.update();
        drive(-10,0.3);
        Arm.setTargetPosition(0);
        TelemeryClear();

        telemetry.addData("Step 25","Turning Right");//Turns to face the other side of the hallway again
        telemetry.update();
        turnTo(0,0.2);
        TelemeryClear();

        telemetry.addData("Step 26","Driving Forward");//Drives to the second ECU fan location = pos 2
        telemetry.update();
        drive(6,0.3);
        TelemeryClear();

        telemetry.addData("Step 27","Turning Right");//Turns to look at pos 2
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 28","Driving Forward");//Drives to pos 2 and grabs it
        telemetry.update();
        drive(7,0.3);
        ClawPos(0.4);
        sleep(500);
        Arm.setTargetPosition(100);
        TelemeryClear();

        telemetry.addData("Step 29","Turning Left");//Turns around to look at the ECU fan zone
        telemetry.update();
        turnTo(90,0.2);
        TelemeryClear();

        telemetry.addData("Step 29","Driving Forward");//Drives and drops the ECU fan off
        telemetry.update();
        drive(17,0.3);
        TelemeryClear();
        ClawPos(0.55);

        telemetry.addData("Step 30","Driving Backwards");//Backs up away from the ECU fan zone
        telemetry.update();
        drive(-10,0.3);
        Arm.setTargetPosition(0);
        TelemeryClear();

        telemetry.addData("Step 31","Turning Right");//Turns to face the end of the hallway again
        telemetry.update();
        turnTo(0,0.2);
        TelemeryClear();

        telemetry.addData("Step 32","Driving Forward");//Drives to the third spot the ECU fan could be = pos 3
        telemetry.update();
        drive(6,0.3);
        TelemeryClear();

        telemetry.addData("Step 33","Turning Right");//Turns to stare at pos 3
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 34","Driving Forward");//Drives and grabs pos 3
        telemetry.update();
        drive(7,0.3);
        ClawPos(0.4);
        sleep(500);
        Arm.setTargetPosition(100);
        TelemeryClear();

        telemetry.addData("Step 35","Driving Backwards");//Drives backwards to old pos
        telemetry.update();
        drive(-10,0.3);
        TelemeryClear();

        telemetry.addData("Step 36","Turning Right");//Turns to go to line up in pos 2 turn point
        telemetry.update();
        turnTo(-180,0.2);
        TelemeryClear();

        telemetry.addData("Step 37","Driving Forwards");//Drives to old pos 2 spot
        telemetry.update();
        drive(6,0.3);
        TelemeryClear();

        telemetry.addData("Step 38","Turning Right");//Turns to stare at the ECU fan zone
        telemetry.update();
        turnTo(-270,0.2);
        TelemeryClear();

        telemetry.addData("Step 39","Driving Forwards");//Drives to the ECU fan zone and drops the ECU fan off
        telemetry.update();
        drive(10,0.3);
        TelemeryClear();
        ClawPos(0.55);

        telemetry.addData("Step 40","Driving Backwards");//Drives backwards to old spot
        telemetry.update();
        drive(10,0.3);
        TelemeryClear();
        Arm.setTargetPosition(0);

        telemetry.addData("Step 41","Turning Right");// turns to face the other hallway wall
        telemetry.update();
        turnTo(-0,0.2);
        TelemeryClear();

        telemetry.addData("Step 42","Driving Forwards");//drives until the auto entry zone
        telemetry.update();
        drive(34.5,0.3);
        TelemeryClear();

        telemetry.addData("Step 43","Turning Right");//Turns to stare at the auto entry zone
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 44","Driving Forwards");//Drives into the entry zone
        telemetry.update();
        drive(17,0.3);
        TelemeryClear();

        telemetry.addData("Step 45","Turning Left");//Turns to look outside of the auto zone
        telemetry.update();
        turnTo(0,0.2);
        TelemeryClear();

        /*
        From here on out it does two cycles that only have one difference to I'll state
        when its the same by referencing the old step
         */
        telemetry.addData("Step 46", "Signaling Ready For Pass-off");//Telling the other bots it's done
        Arm.setTargetPosition(80);
        while(true){
            ClawPos(.4);
            sleep(500);
            ClawPos(.55);
            sleep(500);
            ++test;

            if(test > 5){
                Arm.setTargetPosition(0);
                test = 0;
                break;
            }
        }
        TelemeryClear();



        telemetry.addData("Step 47","Driving Forward");//finds the delivered supply
        telemetry.update();
        TelemeryClear();
        Sensor(36);
        int distanceTravelTicks = LeftWheel.getCurrentPosition();
        double distanceTravelInches = distanceTravelTicks / COUNTS_PER_INCH;
        ClawPos(0.55);
        sleep(400);
        Arm.setTargetPosition(30);

        telemetry.addData("Step 48","Driving Backwards");//Drives back into the auto zone
        telemetry.update();
        drive(-distanceTravelInches,0.3);
        TelemeryClear();

        telemetry.addData("Step 49","Turning Left");//turns to stare inside the hallway
        telemetry.update();
        turnTo(90,0.2);
        TelemeryClear();

        telemetry.addData("Step 50","Driving Forward");//Drives into the hallway
        telemetry.update();
        drive(17,0.3);
        TelemeryClear();

        telemetry.addData("Step 51","Turning Left");//turns toward the safe zone side of the hallway
        telemetry.update();
        turnTo(180,0.2);
        TelemeryClear();

        telemetry.addData("Step 52","Driving Forward");//Drives to the hallway safe zone wall
        telemetry.update();
        drive(62,0.3);
        TelemeryClear();

        telemetry.addData("Step 53","Turning Left");//Turns to stare at the safe zone
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 54","Driving Forwards");//Drives and drops off the supply
        telemetry.update();
        drive(10,0.3);
        TelemeryClear();
        Arm.setTargetPosition(0);
        sleep(500);
        ClawPos(0.4);
        sleep(500);

        telemetry.addData("Step 55","Driving Backwards");//backs out of safe zone
        telemetry.update();
        drive(-10,0.3);
        TelemeryClear();

        telemetry.addData("Step 56","turning Left");//Turns to stare at the entrance hallway wall
        telemetry.update();
        turnTo(0,0.2);
        TelemeryClear();

        telemetry.addData("Step 57","Driving Forwards");//Drives to the end of the hallway entrance wall
        telemetry.update();
        drive(62,0.3);
        TelemeryClear();

        telemetry.addData("Step 57","Turning Right");//turns to look at the auto zone entrance
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 59","Driving Forward");//drives into the entrance zone
        telemetry.update();
        drive(17,0.3);
        TelemeryClear();

        telemetry.addData("Step 60","Turning Left");//Turns to look outside the auto zone
        telemetry.update();
        turnTo(0,0.2);
        TelemeryClear();

        telemetry.addData("Step 61", "Signaling Ready For Pass-off");//Same as step 46
        Arm.setTargetPosition(80);
        while(true){
            ClawPos(.4);
            sleep(500);
            ClawPos(.55);
            sleep(500);
            ++test;

            if(test > 5){
                Arm.setTargetPosition(0);
                test = 0;
                break;
            }
        }
        TelemeryClear();

        telemetry.addData("Step 62", "Driving Forward");//Same as step 47
        telemetry.update();
        TelemeryClear();
        Sensor(36);
         distanceTravelTicks = LeftWheel.getCurrentPosition();
         distanceTravelInches = distanceTravelTicks / COUNTS_PER_INCH;

        telemetry.addData("Step 63","Driving Backwards");//Same as step 48
        telemetry.update();
        drive(-distanceTravelInches,0.3);
        TelemeryClear();

        telemetry.addData("Step 64","Turning Left");//Same as step 49
        telemetry.update();
        turnTo(90,0.2);
        TelemeryClear();

        telemetry.addData("Step 65","Driving Forward");//Same as step 50
        telemetry.update();
        drive(17,0.3);
        TelemeryClear();

        telemetry.addData("Step 66","Turning Left");//Same as step 51
        telemetry.update();
        turnTo(180,0.2);
        TelemeryClear();

        telemetry.addData("Step 67","Driving Forward");//Same as step 52
        telemetry.update();
        drive(62,0.3);
        TelemeryClear();

        telemetry.addData("Step 68","Turning Left");//Same as step 53
        telemetry.update();
        turnTo(-90,0.2);
        TelemeryClear();

        telemetry.addData("Step 69","Driving Forwards");//Same as step 54
        telemetry.update();
        drive(10,0.3);
        TelemeryClear();
        Arm.setTargetPosition(0);
        sleep(500);
        ClawPos(0.4);
        sleep(500);

        telemetry.addData("Step 70","Driving Backwards");//Same as step 55
        telemetry.update();
        drive(-10,0.3);
        TelemeryClear();

        telemetry.addData("Final Step","CELEBRATE!!!");
        telemetry.update();
        while(true){
            ClawPos(.4);
            sleep(500);
            ClawPos(.55);
            sleep(500);
            ++test;

            if(test > 10){
                Arm.setTargetPosition(0);
                test = 0;
                break;
            }
        }

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
        double driveSpeed = 0.3;

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
        turnTo(180, 0.2);
        Arm.setTargetPosition(70);
        sleep(500);

        telemetry.addLine("Step 3: Driving to starting location");
        telemetry.update();
        drive(distanceTravelInches,0.3);
        ClawPos(0.55);

        telemetry.addLine("Autonomous Finished!");
        telemetry.update();
        sleep(2000);*/