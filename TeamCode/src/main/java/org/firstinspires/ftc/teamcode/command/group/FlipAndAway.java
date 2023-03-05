package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.arm.Home;
import org.firstinspires.ftc.teamcode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class FlipAndAway extends SequentialCommandGroup {
    public FlipAndAway(WristSubsystem wrist, ArmSubsystem arm, ClawSubsystem claw){
        addCommands(
                new Release(claw),
                new ParallelCommandGroup(new UnFlip(wrist),
                        new Home(arm))

        );
        addRequirements(wrist, arm, claw);
    }
}
