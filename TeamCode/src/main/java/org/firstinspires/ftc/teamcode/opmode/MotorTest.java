package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Motor Testing")
public class MotorTest extends LinearOpMode {

    public DcMotorSimple motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get ("rightBack");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //initialize robot
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                motorFrontLeft.setPower(1);
            }
            if(gamepad1.b) {
                motorBackRight.setPower(1);
            }
            if(gamepad1.x){
                motorBackLeft.setPower(1);
            }
            if (gamepad1.y){
                motorFrontRight.setPower(1);
            }
        }
        //frontLeft is backright
        //
    }
}
