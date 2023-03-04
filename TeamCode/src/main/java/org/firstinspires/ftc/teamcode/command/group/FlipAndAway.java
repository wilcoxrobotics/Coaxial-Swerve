package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class FlipAndAway extends SequentialCommandGroup {
    public FlipAndAway(WristSubsystem wrist, ArmSubsystem arm, WristSubsystem wrist1){
        addCommands(
                new Flip(wrist1),
                new Flip(wrist),
                new Away(arm)
        );
        addRequirements(wrist, arm);
    }
}
