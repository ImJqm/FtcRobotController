package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp(name = "FieldCentric")
public class TeleOpFieldCentric extends OpMode {


    DcMotor motorFrL;
    DcMotor motorFrR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor vMotor1;
    DcMotor vMotor2;
    IMU imu;
    Servo servo1;
    Servo servo2;
    Servo servo3;

    double yJacketTicks = 384.5;
    double nextTarget;
    boolean lastMovementx;
    boolean currMovementx;
    boolean downPositionx;

    boolean lastMovement2a;
    boolean currMovement2a;
    boolean downPosition2a;

    boolean lastMovementb;
    boolean currMovementb;
    boolean downPositionb;

    boolean lastMovementlb;
    boolean currMovementlb;
    boolean downPositionlb;

    boolean toggle;

    boolean lastMovementrb;
    boolean currMovementrb;
    boolean downPositionrb;

    int count;

    DcMotor[] vArr = null;

    @Override
    public void init() {

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT ));
        imu.initialize(parameters);

        motorFrL = hardwareMap.get(DcMotor.class,"motor0");
        motorFrR = hardwareMap.get(DcMotor.class,"motor1");
        motorBL = hardwareMap.get(DcMotor.class,"motor2");
        motorBR = hardwareMap.get(DcMotor.class,"motor3");

        vMotor1 = hardwareMap.get(DcMotor.class, "vmotor1");
        vMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vMotor2 = hardwareMap.get(DcMotor.class, "vmotor2");
        vMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        toggle = false;

        lastMovement2a = false;
        currMovement2a = false;
        downPosition2a = true;

        lastMovementx = false;
        currMovementx = false;
        downPositionx = true;

        lastMovementb = false;
        currMovementb = false;
        downPositionb = true;

        lastMovementlb = false;
        currMovementlb = false;
        downPositionlb = true;

        lastMovementrb = false;
        currMovementrb = false;
        downPositionrb = true;

        motorFrR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        vMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        vMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        vArr = new DcMotor[]{vMotor1,vMotor2};
        servo3.setDirection(Servo.Direction.REVERSE);
        //servo1.setDirection(Servo.Direction.REVERSE);
        servo1.setPosition(0.49);
        servo2.setPosition(0.05);
        servo3.setPosition(0.3);

        count =0;

    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }
        /*]if (gamepad1.b) {
            encoder(8);
        }*/
        telemetry.addData("RT", (int)(gamepad1.right_trigger*3));
        telemetry.addData("Servo3 Pos:", servo3.getPosition());

        lastMovementx = currMovementx;
        currMovementx = gamepad1.x;
        if(currMovementx && !lastMovementx){
            downPositionx = !downPositionx;
            servo2.setPosition(0.5);
            if(downPositionx){
                //servo1.setPosition(0.25);
                servo1.setPosition(0.49);
                vArr[0].setTargetPosition(0);
                vArr[0].setPower(0.4);
                vArr[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else{
                //servo1.setPosition(1);
                vcode(7, 0);
            }
        }
        telemetry.addData("LB: ", gamepad1.left_bumper);
        lastMovementlb = currMovementlb;
        currMovementlb = gamepad1.left_bumper;
        if(currMovementlb && !lastMovementlb){
            downPositionlb = !downPositionlb;

            if(downPositionlb) {
                servo3.setPosition(0.1);
                //servo1.setPosition(1);
            } else {
                servo3.setPosition(0.4);
                //servo1.setPosition(0.49);//decreasing makes it more
            }
        }

        //telemetry.addData("LB: ", gamepad1.left_bumper);
        lastMovementrb = currMovementrb;
        currMovementrb = gamepad1.right_bumper;
        if(currMovementrb && !lastMovementrb && vArr[0].getCurrentPosition()>=455){
            downPositionrb = !downPositionrb;
            servo2.setPosition(0.5);
            if(downPositionrb) {
                //servo3.setPosition(0.1);
                servo1.setPosition(0.49);
            } else {
                //servo3.setPosition(0.4);
                servo1.setPosition(1);//decreasing makes it more
            }
        }

        if (gamepad1.dpad_down) {
            servo2.setPosition(0.67);
        }
        if (gamepad1.dpad_right) {
            servo2.setPosition(0.55);
        }
        if (gamepad1.dpad_up) {
            servo2.setPosition(0.05);
        }

        //if (gamepad1.left_trigger>=70)

        /*if (gamepad1.left_trigger>=.67) {
            servo2.setPosition(0.67);
        } else if (gamepad1.left_trigger<=0.05){
            servo2.setPosition(0.05);
        } else {
            servo2.setPosition(gamepad1.left_trigger);
        }*/
        telemetry.addData("B", gamepad1.b);
        //lastMovementb = currMovementb;
        //currMovementb = gamepad1.b;
        if (gamepad1.right_trigger==0) {
            vArr[1].setTargetPosition(0);
            vArr[1].setPower(0.3);
            vArr[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
                vcode((int)(gamepad1.right_trigger*3),1);
        }



        sPower(0, 455);
        sPower(1, 200);
        telemetry.addData("V2 Pos:", vArr[1].getCurrentPosition());
        telemetry.addData("2 is busy", vArr[1].isBusy());
        telemetry.addData("Target Pos:", vArr[0].getTargetPosition());
        telemetry.addData("Is Busy:", vArr[0].isBusy());
        telemetry.addData("Current Pos",vArr[0].getCurrentPosition());
        telemetry.update();

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y* Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX*1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        lastMovement2a = currMovement2a;
        currMovement2a = gamepad1.a;
        if(currMovement2a && !lastMovement2a){
            downPosition2a = !downPosition2a;
            if(downPosition2a) {
                toggle = false;
            } else {
                toggle = true;
            }
        }


        if (toggle) {
            motorFrL.setPower(frontLeftPower*0.25);
            motorFrR.setPower((frontRightPower*0.97)*0.25);
            motorBL.setPower(backLeftPower*0.25);
            motorBR.setPower(backRightPower*0.25);
        } else {
            motorFrL.setPower(frontLeftPower);
            motorFrR.setPower(frontRightPower*0.97);
            motorBL.setPower(backLeftPower);
            motorBR.setPower(backRightPower);
        }


    }
    public void sPower(int mid, int min) {
        if(!vArr[mid].isBusy() && vArr[0].getCurrentPosition()<=min) {
            vArr[mid].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            vArr[mid].setPower(0);
            if (mid==0) {
                vArr[mid].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (mid ==1) {
                telemetry.addData("2 FLOAT",1);
            }


        } else {
            vArr[mid].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (mid ==1) {
                telemetry.addData("2 BRAKE",1);
            }

        }
    }
    public void vcode(int turnage, int mid) {
        nextTarget = yJacketTicks*turnage;
        vArr[mid].setTargetPosition((int)nextTarget);
        vArr[mid].setPower(0.3);
        vArr[mid].setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



}
