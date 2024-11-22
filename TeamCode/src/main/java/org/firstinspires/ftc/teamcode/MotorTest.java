package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "test")
public class MotorTest extends OpMode {


    DcMotor motorFrL;
    DcMotor motorFrR;
    DcMotor motorBL;
    DcMotor motorBR;

    @Override
    public void init() {

        motorFrL = hardwareMap.get(DcMotor.class,"motor0");
        motorFrR = hardwareMap.get(DcMotor.class,"motor1");
        motorBL = hardwareMap.get(DcMotor.class,"motor2");
        motorBR = hardwareMap.get(DcMotor.class,"motor3");

        motorFrR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrL.setPower(frontLeftPower);
        motorFrR.setPower(frontRightPower);
        motorBL.setPower(backLeftPower);
        motorBR.setPower(backRightPower);

    }
}
