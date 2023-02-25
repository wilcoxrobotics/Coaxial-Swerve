package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.Junction;

public class GrabRotateLift extends SequentialCommandGroup {
    public GrabRotateLift(ClawSubsystem claw, ArmSubsystem arm, LiftSubsystem lift, WristSubsystem wrist, Junction junction) {
        addCommands(
                new Grab(claw),
                new ParallelCommandGroup(
                        new SetJunction(lift, junction),
                        new DelayedCommand(
                                new FlipAndAway(wrist, arm),
                                200
                        )
                )
        );
        addRequirements(claw, arm, wrist, lift);
    }
}
