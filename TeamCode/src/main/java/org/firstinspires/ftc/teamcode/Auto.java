package org.firstinspires.ftc.teamcode;
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

    @Override
    public void runOpMode() throws InterruptedException {
    initHardware();
    double x = getDistance();
    telemetry.addData("Distance:", x);
    telemetry.update();
    while(opModeIsActive()){
        if(TwoMeter_Sensor.getDistance(DistanceUnit.INCH) > 10){
            RightWheel.setPower(.4);
            LeftWheel.setPower(.4);
        }
    }
    }
}
