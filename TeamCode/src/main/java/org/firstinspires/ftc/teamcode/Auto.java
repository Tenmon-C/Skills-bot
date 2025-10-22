package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous (name = "Auto")
public class Auto extends Base {
int LWheelCorrection;
int RWheelCorrection;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initHardware();

    waitForStart();
    while(opModeIsActive()){

        while(getDistance() > 20){
            double x = getDistance();
            telemetry.addData("Distance:", x);
            telemetry.update();
            LWheelCorrection += 10;
            RWheelCorrection += 10;
            LeftWheel.setTargetPosition(LWheelCorrection);
            RightWheel.setTargetPosition(RWheelCorrection);


        }
        ResetMotorEncoders();


        telemetry.addData("RightWheel:",RightWheel);
        telemetry.addData("LeftWheel:",LeftWheel);
        telemetry.addData("Arm:",Arm);
        telemetry.update();
    }
    }
}
