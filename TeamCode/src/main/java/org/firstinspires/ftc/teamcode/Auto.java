package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Ex;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@Autonomous (name = "Auto")
public class Auto extends Base {

    int LWheelCorrection;
    int RWheelCorrection;
    boolean sensor = true;

    boolean turn = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initHardware();
        while (!isStarted()) {
            T();
            RightWheel.setPower(0);
            LeftWheel.setPower(0);
            Arm.setPower(0);
        }

        waitForStart();
        ResetMotorEncoders();
        Arm.setPower(0.5);
        Sensor();
        ResetMotorEncoders();


        while (opModeIsActive()) {
            T();
        }


    }

    public void Sensor() {
        if (getDistance() > 20 && sensor) {
            while (getDistance() > 20) {
                double x = getDistance();
                telemetry.addData("Distance:", x);
                telemetry.update();
                LWheelCorrection += 10;
                RWheelCorrection += 10;
                LeftWheel.setTargetPosition(LWheelCorrection);
                RightWheel.setTargetPosition(RWheelCorrection);

            }
        } else {
            sensor = false;
        }
    }
        }






