package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="unpowered servo")
public class ClawTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw");
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a) {
                claw.setPosition(0.65);
            }
            if(gamepad1.b){
                claw.setPosition(0.4);
            }
        }
    }
}
