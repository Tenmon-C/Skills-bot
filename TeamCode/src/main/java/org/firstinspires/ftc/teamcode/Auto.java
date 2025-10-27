package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous (name = "Auto")
public class Auto extends Base {

    int LWheelCorrection;
    int RWheelCorrection;
    boolean sensor = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initHardware();

        while (!isStarted()) {
            T();
            telemetry.addData("Mode","waiting for start");
            telemetry.addData("IMU Calibration Status", imu.getRobotYawPitchRollAngles().toString());
            telemetry.update();
        }

        waitForStart();

        Sensor();
        ResetMotorEncoders();

        RightWheel.setTargetPosition(-400);
        LeftWheel.setTargetPosition(400);
        while (RightWheel.getCurrentPosition() > RightWheel.getTargetPosition()) {
        }
        RightWheel.setPower(0);
        LeftWheel.setPower(0);

        while (opModeIsActive()) {
            T();
            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            double yawAngle = robotOrientation.getYaw(AngleUnit.RADIANS);
            telemetry.addData("Yaw (Radians)", "%.2f", yawAngle);
            telemetry.update();
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




