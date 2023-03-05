//package org.firstinspires.ftc.teamcode.command.group;
//
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import org.firstinspires.ftc.teamcode.command.arm.Away;
//import org.firstinspires.ftc.teamcode.command.wrist.Flip;
//import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
//import org.firstinspires.ftc.teamcode.command.group.FlipAndAway;
//
//import java.util.function.BooleanSupplier;
//
//public class FlipAndAwayCheck extends SequentialCommandGroup {
//    public FlipAndAwayCheck(WristSubsystem wrist, ArmSubsystem arm){
//        addCommands(
//                new ConditionalCommand(new FlipAndAway(wrist, arm), new InstantCommand(), () -> ClawSubsystem.pos)
//        );
//        addRequirements(wrist, arm);
//    }
//}