package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.arm.Home;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class UnFlipAndHome extends SequentialCommandGroup {
    public UnFlipAndHome(WristSubsystem wrist, ArmSubsystem arm, WristSubsystem wrist1){
        addCommands(
                new UnFlip(wrist1),
                new ParallelCommandGroup(
                        new UnFlip(wrist),
                        new Home(arm)
                )
        );
        addRequirements(wrist, arm);
    }
}
