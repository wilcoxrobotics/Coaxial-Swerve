package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.arm.Mid;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.Junction;

public class GreatReset extends SequentialCommandGroup {
    public GreatReset( ArmSubsystem arm, WristSubsystem wrist, WristSubsystem wrist1, LiftSubsystem lift) {
        addCommands(
                new UnFlipAndHome(wrist, arm, wrist1),
                new Mid(arm),
                new UnFlip(wrist1),
                new DelayedCommand(new SetJunction(lift, Junction.NONE), 200)
        );
    }
}
