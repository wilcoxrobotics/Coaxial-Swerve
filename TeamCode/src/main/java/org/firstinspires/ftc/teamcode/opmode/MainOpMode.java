package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.arm.Home;
import org.firstinspires.ftc.teamcode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.drive.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.command.drive.DriveSlowMode;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.lift.UpdateLift;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.util.Junction;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@TeleOp
public class MainOpMode extends BaseOpMode {
    private DriveRobotCentric robotCentricDrive;
    private DriveSlowMode slowMode;
    private UpdateLift updateLift;

    @Override
    public void initialize() {
        super.initialize();

        // drive

        robotCentricDrive = new DriveRobotCentric(drive, gamepadEx1::getRightX,
                gamepadEx1::getLeftY, gamepadEx1::getLeftX );

        slowMode = new DriveSlowMode(drive, gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX);
        //drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
//        updateLift = new UpdateLift(lift);

        gb1(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(slowMode);

        gb1(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new Grab(claw), new Release(claw));


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

        gb1(GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(new Flip(wrist), new UnFlip(wrist));

        register(drive, lift, claw, arm, wrist);
        drive.setDefaultCommand(robotCentricDrive);
//        lift.setDefaultCommand(updateLift);

    }
}
