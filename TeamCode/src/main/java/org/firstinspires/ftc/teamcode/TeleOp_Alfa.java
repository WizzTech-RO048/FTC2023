package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.*;


@TeleOp(name="TeleOp_Alfa", group="TeleOp")
public class TeleOp_Alfa extends LinearOpMode {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    @Override
    public void runOpMode()  throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft = hardwareMap.dcMotor.get("left_front");
        backLeft = hardwareMap.dcMotor.get("left_rear");
        frontRight = hardwareMap.dcMotor.get("right_front");
        backRight = hardwareMap.dcMotor.get("right_rear");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //frontLeft.setDirection(DcMotor.Direction.REVERSE);
        //backLeft.setDirection(DcMotor.Direction.REVERSE);

        //future to do check which motors must be reversed

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double theta =  Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin),Math.abs(cos));

            double leftFront = power * cos/max + turn;
            double rightFront = power * sin/max - turn;
            double leftRear = power * sin/max + turn;
            double rightRear = power * cos/max - turn;

            if((power + Math.abs(turn)) > 1){
                leftFront /= power + turn;
                rightFront /= power + turn;
                leftRear /= power + turn;
                rightRear /= power + turn;
            }

            frontLeft.setPower(leftFront);
            frontRight.setPower(rightFront);
            backLeft.setPower(leftRear);
            backRight.setPower(rightRear);

            telemetry.addData("leftfront Power", leftFront);
            telemetry.addData("rightfront Power", rightFront);
            telemetry.addData("leftrear Power", leftRear);
            telemetry.addData("rightRear Power", rightRear);
        }
    }
}
