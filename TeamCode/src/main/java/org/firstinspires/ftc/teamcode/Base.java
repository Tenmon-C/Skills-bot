package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public abstract class Base extends LinearOpMode {

    DistanceSensor TwoMeter_Sensor;
    DcMotorEx RightWheel;
    DcMotorEx Arm;
    DcMotorEx LeftWheel;
    Servo RightClaw;
    Servo LeftClaw;
    IMU imu;

    public void initHardware(){
        TwoMeter_Sensor = hardwareMap.get(DistanceSensor.class,"Sensor");
        RightWheel = hardwareMap.get(DcMotorEx.class,"RightWheel");
        LeftWheel = hardwareMap.get(DcMotorEx.class,"LeftWheel");
        Arm = hardwareMap.get(DcMotorEx.class,"Arm");
        RightClaw = hardwareMap.get(Servo.class,"RightClaw");
        LeftClaw = hardwareMap.get(Servo.class,"LeftClaw");
        imu = hardwareMap.get(IMU.class,"Imu");

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

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection,usbFacingDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
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
    public void T(){
        telemetry.addData("RightWheel", RightWheel.getCurrentPosition());
        telemetry.addData("LeftWheel", LeftWheel.getCurrentPosition());
        telemetry.addData("Arm", Arm.getCurrentPosition());
        telemetry.update();


    }



}
