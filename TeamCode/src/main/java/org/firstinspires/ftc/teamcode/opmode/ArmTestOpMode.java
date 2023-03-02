package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.arm.Home;
import org.firstinspires.ftc.teamcode.command.drive.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.util.Junction;

@TeleOp(name="ARM TEST", group = "TeleOp")
public class ArmTestOpMode extends ArmTestBaseOpMode{


    @Override
    public void initialize() {
        super.initialize();

        gb1(GamepadKeys.Button.A)
                .whenPressed(new Home(arm));
        gb2(GamepadKeys.Button.B)
                .whenPressed(new Away(arm));

        register(arm);
    }


}
