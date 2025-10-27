package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous (name = "IMUTest")
public class IMUTest extends Base{

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initHardware();
        while(!isStarted()){
            telemetry.addData("Mode","waiting for start");
            telemetry.addData("IMU Calibration Status", imu.getRobotYawPitchRollAngles().toString());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){
            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            double yawAngle = robotOrientation.getYaw(AngleUnit.RADIANS);
            telemetry.addData("Yaw (Radians)", "%.2f", yawAngle);
            telemetry.update();
        }

    }
}
