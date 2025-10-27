package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "EncoderTest")
public class EncoderTest extends Base{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initHardware();
        waitForStart();

        while(opModeIsActive()){
            RightWheel.setPower(0);
            LeftWheel.setPower(0);
            Arm.setPower(0);
            telemetry.addData("RightWheel", RightWheel.getCurrentPosition());
            telemetry.addData("LeftWheel", LeftWheel.getCurrentPosition());
            telemetry.addData("Arm", Arm.getCurrentPosition());
            telemetry.update();
        }


    }

    }



