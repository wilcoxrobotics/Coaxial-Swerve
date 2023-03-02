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
import org.firstinspires.ftc.teamcode.util.Junction;

@TeleOp(name="Claw Test", group = "TeleOp")
public class ClawOpMode extends ClawBaseOpMode{
    @Override
    public void initialize() {
        super.initialize();
        gb2(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new Grab(claw), new Release(claw));
        gb2(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new Away(arm), new Home(arm));

        register(claw,arm);
    }
}
