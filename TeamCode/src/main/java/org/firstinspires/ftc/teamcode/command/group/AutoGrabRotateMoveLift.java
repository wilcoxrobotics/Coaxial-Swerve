package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;

public class AutoGrabRotateMoveLift extends SequentialCommandGroup {
    public AutoGrabRotateMoveLift( ArmSubsystem arm, WristSubsystem wrist, WristSubsystem wrist1, LiftSubsystem lift, Junction junction) {
        addCommands(
                new GrabRotateLift( arm, lift, wrist, wrist1, junction)
        );
        addRequirements( arm, wrist, lift,wrist1);
    }

}
