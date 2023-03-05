package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.arm.Home;
import org.firstinspires.ftc.teamcode.command.arm.Mid;
import org.firstinspires.ftc.teamcode.command.drive.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.command.drive.DriveSlowMode;
import org.firstinspires.ftc.teamcode.command.group.AutoGrabRotateMoveLift;
import org.firstinspires.ftc.teamcode.command.group.FlipAndAway;
import org.firstinspires.ftc.teamcode.command.group.UnFlipAndHome;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.util.Junction;


@TeleOp
public class MainOpMode extends BaseOpMode {

    @Override
    public void initialize() {
        super.initialize();

        DriveRobotCentric robotCentricDrive = new DriveRobotCentric(drive, gamepadEx1::getLeftX,
                gamepadEx1::getLeftY, gamepadEx1::getRightX);

        DriveSlowMode slowMode = new DriveSlowMode(drive, gamepadEx1::getLeftX,
                gamepadEx1::getLeftY, gamepadEx1::getRightX);

        gb1(GamepadKeys.Button.LEFT_STICK_BUTTON).whileHeld(slowMode);
        gb1(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new Flip(wrist1), new UnFlip(wrist1));
        gb1(GamepadKeys.Button.A).whenPressed(new SetJunction(lift, Junction.NONE));
        gb1(GamepadKeys.Button.X).whenPressed(new SetJunction(lift, Junction.LOW));
        gb1(GamepadKeys.Button.B).whenPressed(new SetJunction(lift, Junction.MEDIUM));
        gb1(GamepadKeys.Button.Y).whenPressed(new SetJunction(lift, Junction.HIGH));
        gb1(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new FlipAndAway(wrist, arm,wrist1), new UnFlipAndHome(wrist, arm, wrist1));
        gb1(GamepadKeys.Button.START).whenPressed(new Mid(arm));
        gb1(GamepadKeys.Button.DPAD_UP).whenPressed(new AutoGrabRotateMoveLift(arm, wrist, wrist1, lift, Junction.HIGH));
        gb1(GamepadKeys.Button.DPAD_LEFT).whenPressed(new AutoGrabRotateMoveLift(arm,wrist,wrist1,lift,Junction.MEDIUM));
        gb1(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new AutoGrabRotateMoveLift(arm,wrist,wrist1,lift,Junction.LOW));
        gb1(GamepadKeys.Button.DPAD_DOWN).whenPressed(new AutoGrabRotateMoveLift(arm,wrist,wrist1, lift, Junction.NONE));

        //gb2 controller settings
        gb2(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new Flip(wrist1), new UnFlip(wrist1));
        gb2(GamepadKeys.Button.DPAD_UP).toggleWhenPressed(new UnFlip(wrist1), new Flip(wrist1));
        gb2(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new Away(arm), new Home(arm));
        gb2(GamepadKeys.Button.A).whenPressed(new SetJunction(lift, Junction.NONE));
        gb2(GamepadKeys.Button.X).whenPressed(new SetJunction(lift, Junction.LOW));
        gb2(GamepadKeys.Button.B).whenPressed(new SetJunction(lift, Junction.MEDIUM));
        gb2(GamepadKeys.Button.Y).whenPressed(new SetJunction(lift, Junction.HIGH));
        gb2(GamepadKeys.Button.START).whenPressed(new Mid(arm));
        register(drive, lift, wrist1, arm, wrist);
        drive.setDefaultCommand(robotCentricDrive);
    }

}
