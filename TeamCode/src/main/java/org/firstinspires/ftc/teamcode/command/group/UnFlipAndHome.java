package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class UnFlipAndHome extends SequentialCommandGroup {
    public UnFlipAndHome(WristSubsystem wrist, ArmSubsystem arm, ClawSubsystem claw){
        addCommands(
                new Release(claw),
                new ParallelCommandGroup(new Flip(wrist),
                        new Away(arm))
        );
        addRequirements(wrist, arm, claw);
    }
}
