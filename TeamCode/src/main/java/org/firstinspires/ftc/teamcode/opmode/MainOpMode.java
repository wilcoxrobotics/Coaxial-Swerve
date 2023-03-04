package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.arm.Home;
import org.firstinspires.ftc.teamcode.command.arm.Mid;
import org.firstinspires.ftc.teamcode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.drive.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.command.drive.DriveSlowMode;
import org.firstinspires.ftc.teamcode.command.group.FlipAndAway;
import org.firstinspires.ftc.teamcode.command.group.GrabRotateLift;
import org.firstinspires.ftc.teamcode.command.group.UnFlipAndHome;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.lift.UpdateLift;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;

import java.util.Objects;

@TeleOp
public class MainOpMode extends BaseOpMode {
    private DriveRobotCentric robotCentricDrive;
    private DriveSlowMode slowMode;
    private UpdateLift updateLift;

    @Override
    public void initialize() {
        super.initialize();

        robotCentricDrive = new DriveRobotCentric(drive, gamepadEx1::getLeftX,
                gamepadEx1::getLeftY, gamepadEx1::getRightX);

        slowMode = new DriveSlowMode(drive, gamepadEx1::getLeftX,
                gamepadEx1::getLeftY, gamepadEx1::getRightX);

        gb1(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(slowMode);

        gb1(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new Flip(wrist1), new UnFlip(wrist1));

        gb1(GamepadKeys.Button.A)
                .whenPressed(new SetJunction(lift, Junction.NONE));
        gb1(GamepadKeys.Button.X)
                .whenPressed(new SetJunction(lift, Junction.LOW));
        gb1(GamepadKeys.Button.B)
                .whenPressed(new SetJunction(lift, Junction.MEDIUM));
        gb1(GamepadKeys.Button.Y)
                .whenPressed(new SetJunction(lift, Junction.HIGH));

        gb1(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new Away(arm), new Home(arm));

        gb1(GamepadKeys.Button.START).whenPressed(new Mid(arm));

        gb1(GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(new Flip(wrist), new UnFlip(wrist));
        gb2(GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(new Flip(wrist), new UnFlip(wrist));

        gb2(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new FlipAndAway(wrist, arm, wrist1), new UnFlipAndHome(wrist, arm, wrist1));


        gb2(GamepadKeys.Button.A)
                .whenPressed(new GrabRotateLift(arm, lift, wrist, Junction.HIGH));

//        if(colorCheck(colorSensor.red()) && Objects.equals(ClawSubsystem.mode, "open")){
//            new Grab(claw);
//        }
//
//        if(colorCheck(colorSensor.blue()) && Objects.equals(ClawSubsystem.mode, "open")){
//            new Grab(claw);
//        }


        register(drive, lift, wrist1, arm, wrist);
        drive.setDefaultCommand(robotCentricDrive);
//        lift.setDefaultCommand(updateLift);

    }

    public boolean colorCheck(int color){
        if(color>2400 && color<2600){
            return true;
        } else {
            return false;
        }
    }
}
