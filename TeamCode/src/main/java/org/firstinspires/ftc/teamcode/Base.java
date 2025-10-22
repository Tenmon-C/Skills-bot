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
public abstract class Base extends LinearOpMode {

    DistanceSensor TwoMeter_Sensor;
    DcMotor RightWheel;
    DcMotor Arm;
    DcMotor LeftWheel;
    Servo RightClaw;
    Servo LeftClaw;
    public void initHardware(){
        TwoMeter_Sensor = hardwareMap.get(DistanceSensor.class,"Sensor");
        RightWheel = hardwareMap.get(DcMotor.class,"RightWheel");
        LeftWheel = hardwareMap.get(DcMotor.class,"LeftWheel");
        Arm = hardwareMap.get(DcMotor.class,"Arm");
        RightClaw = hardwareMap.get(Servo.class,"RightClaw");
        LeftClaw = hardwareMap.get(Servo.class,"LeftClaw");

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0); //shouldn't be necessary but just in case
        Arm.setPower(0);//shouldn't be necessary but just in case
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); //put motor in run to target mode
        Arm.setPower(0.5);

        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setTargetPosition(0); //shouldn't be necessary but just in case
        RightWheel.setPower(0);//shouldn't be necessary but just in case
        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION); //put motor in run to target mode
        RightWheel.setPower(0.3);

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftWheel.setTargetPosition(0); //shouldn't be necessary but just in case
        LeftWheel.setPower(0);//shouldn't be necessary but just in case
        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION); //put motor in run to target mode
        LeftWheel.setPower(0.3);

    }
public double getDistance(){
return TwoMeter_Sensor.getDistance(DistanceUnit.INCH);
}

    public void ResetMotorEncoders(){
        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setTargetPosition(0); //shouldn't be necessary but just in case
        RightWheel.setPower(0);//shouldn't be necessary but just in case
        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION); //put motor in run to target mode
        RightWheel.setPower(0.3);

        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftWheel.setTargetPosition(0); //shouldn't be necessary but just in case
        LeftWheel.setPower(0);//shouldn't be necessary but just in case
        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION); //put motor in run to target mode
        LeftWheel.setPower(0.3);
    }
}
