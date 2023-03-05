package org.firstinspires.ftc.teamcode.command.arm;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class DownAndHome extends SequentialCommandGroup {
    public DownAndHome( ArmSubsystem arm, WristSubsystem wrist1){
        addCommands(
                new Down(arm),
                new UnFlip(wrist1),
                new Mid(arm)
        );
        addRequirements( arm, wrist1);
    }
}