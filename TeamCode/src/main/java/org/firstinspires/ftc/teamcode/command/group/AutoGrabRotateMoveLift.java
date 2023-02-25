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
    public AutoGrabRotateMoveLift(ClawSubsystem claw, ArmSubsystem arm, WristSubsystem wrist, LiftSubsystem lift) {
        addCommands(
                new GrabRotateLift(claw, arm, lift, wrist, Junction.HIGH)
        );
        addRequirements(claw, arm, wrist, lift);
    }

}
