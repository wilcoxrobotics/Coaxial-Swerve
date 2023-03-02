package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="Test Arm Normal", group = "TeleOp")
public class TestArmNormal extends LinearOpMode {

    ServoImplEx armL, armR;
    protected GamepadEx gamepadEx1;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx1 = new GamepadEx(gamepad1);
        armR = (ServoImplEx) hardwareMap.servo.get("armR");
        armL = (ServoImplEx) hardwareMap.servo.get("armL");
        armR.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                armR.setPosition(0.4);
                armL.setPosition(0.4);
            } else if (gamepad1.b) {
                armR.setPosition(0);
                armL.setPosition(0);
            }
        }

    }

    protected GamepadButton gb1(GamepadKeys.Button button){
        return gamepadEx1.getGamepadButton(button);
    }

}
