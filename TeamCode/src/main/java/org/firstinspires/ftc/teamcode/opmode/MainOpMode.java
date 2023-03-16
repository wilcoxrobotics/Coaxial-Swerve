package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.arm.DownAndHome;
import org.firstinspires.ftc.teamcode.command.arm.Home;
import org.firstinspires.ftc.teamcode.command.arm.Mid;
import org.firstinspires.ftc.teamcode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.drive.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.command.drive.DriveSlowMode;
import org.firstinspires.ftc.teamcode.command.group.AutoGrabRotateMoveLift;
import org.firstinspires.ftc.teamcode.command.group.FlipAndAway;
import org.firstinspires.ftc.teamcode.command.group.GreatReset;
import org.firstinspires.ftc.teamcode.command.group.UnFlipAndHome;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.util.Junction;


@TeleOp
public class MainOpMode extends BaseOpMode {

    @Override
    public void initialize() {
        super.initialize();
        new Release(claw);
        new Mid(arm);
        new UnFlip(wrist);

        DriveRobotCentric robotCentricDrive = new DriveRobotCentric(drive, gamepadEx1::getLeftX,
                gamepadEx1::getRightX, gamepadEx1::getLeftY );

        DriveSlowMode slowMode = new DriveSlowMode(drive, gamepadEx1::getLeftX,
                gamepadEx1::getLeftY, gamepadEx1::getRightX);

        gb1(GamepadKeys.Button.START).toggleWhenPressed(new AutoGrabRotateMoveLift(color,autoDrive,arm,wrist,claw,lift, Junction.HIGH), new GreatReset(autoDrive, arm, wrist, claw,lift));


        //forward is
        /*
        new FollowTrajectorySequenceCommand(autoDrive, autoDrive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .back(30)
                .build()
        )
        */

        gb1(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new Grab(claw), new Release(claw));
        gb1(GamepadKeys.Button.A).whenPressed(new SetJunction(lift, Junction.NONE));
        gb1(GamepadKeys.Button.X).whenPressed(new SetJunction(lift, Junction.LOW));
        gb1(GamepadKeys.Button.B).whenPressed(new SetJunction(lift, Junction.MEDIUM));
        gb1(GamepadKeys.Button.Y).whenPressed(new SetJunction(lift, Junction.HIGH));
        gb1(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new FlipAndAway(wrist, arm, claw), new UnFlipAndHome(wrist, arm, claw));
//        gb1(GamepadKeys.Button.).whenPressed(new GreatReset(arm,wrist, claw,lift));
        //gb1(GamepadKeys.Button.DPAD_UP).whenPressed(new AutoGrabRotateMoveLift(autoDrive,arm, wrist, claw, lift, Junction.HIGH));
        gb1(GamepadKeys.Button.DPAD_LEFT).whenPressed(new Mid(arm));
        //gb1(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new AutoGrabRotateMoveLift(,arm,wrist,claw,lift,Junction.LOW));
        gb1(GamepadKeys.Button.DPAD_DOWN).whenPressed(new DownAndHome(arm,claw));
        gb1(GamepadKeys.Button.LEFT_STICK_BUTTON).toggleWhenPressed(new Flip(wrist), new UnFlip(wrist));
        //gb2 controller settings
        gb2(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new Grab(claw), new Release(claw));
        gb2(GamepadKeys.Button.DPAD_UP).toggleWhenPressed(new Release(claw), new Grab(claw));
//        new UnFlip(wrist1), new Flip(wrist1)
        gb2(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new Away(arm), new Home(arm));
        gb2(GamepadKeys.Button.A).whenPressed(new SetJunction(lift, Junction.NONE));
        gb2(GamepadKeys.Button.X).whenPressed(new SetJunction(lift, Junction.LOW));
        gb2(GamepadKeys.Button.B).whenPressed(new SetJunction(lift, Junction.MEDIUM));
        gb2(GamepadKeys.Button.Y).whenPressed(new SetJunction(lift, Junction.HIGH));
        gb2(GamepadKeys.Button.DPAD_DOWN).whenPressed(new Mid(arm));
        //gb2(GamepadKeys.Button.START).whenPressed(new GreatReset(arm,wrist,claw,lift));
        register(drive, lift, claw, arm, wrist);
        drive.setDefaultCommand(robotCentricDrive);
    }

}
