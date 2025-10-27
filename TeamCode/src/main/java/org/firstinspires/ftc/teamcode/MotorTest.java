package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp
public class MotorTest extends Base{
    int LWheelTest;
    int RWheelTest;
    int ArmTest;
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initHardware();


        waitForStart();

        while(opModeIsActive()){
            boolean Abutton = gamepad1.a;
            boolean Bbutton = gamepad1.b;
            boolean Ybutton = gamepad1.y;
            boolean Xbutton = gamepad1.x;
            double leftX = gamepad1.left_stick_x;;
            double leftY =  gamepad1.left_stick_y;;
            double rightX = gamepad1.right_stick_x;
            leftX *= Math.abs(leftX);
            rightX *= Math.abs(rightX);
            leftY *= Math.abs(leftY);
            T();
            if(Abutton){
                LWheelTest += 1;
                RWheelTest += 1;
                RightWheel.setTargetPosition(RWheelTest);
                LeftWheel.setTargetPosition(LWheelTest);
            }
            if(Bbutton){
                ArmTest += 1;
                Arm.setTargetPosition(ArmTest);
            }
            if(Xbutton){
                RWheelTest -= 1;
                LWheelTest -= 1;
                RightWheel.setTargetPosition(RWheelTest);
                LeftWheel.setTargetPosition(LWheelTest);

            }
            if(Ybutton){
                ArmTest -= 1;
                Arm.setTargetPosition(ArmTest);

            }

    }

    }

}
