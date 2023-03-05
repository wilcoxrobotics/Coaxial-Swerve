package org.firstinspires.ftc.teamcode.command.arm;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class DownAndHome extends SequentialCommandGroup {
    public DownAndHome( ArmSubsystem arm, ClawSubsystem claw){
        addCommands(
                new Down(arm),
                new Release(claw),
                new Mid(arm)
        );
        addRequirements( arm, claw);
    }
}