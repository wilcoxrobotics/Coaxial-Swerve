package org.firstinspires.ftc.teamcode.command.group;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.command.arm.Mid;
import org.firstinspires.ftc.teamcode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.util.Junction;
@Config
public class GreatReset extends SequentialCommandGroup {
    public static int backward = 23;
    public GreatReset(SampleMecanumDrive autoDrive, ArmSubsystem arm, WristSubsystem wrist, ClawSubsystem claw, LiftSubsystem lift) {
        addCommands(
                new ParallelCommandGroup(
                        new FollowTrajectorySequenceCommand(autoDrive, autoDrive.trajectoryBuilder(new Pose2d(0, 0, 0))
                                .back(backward)
                                .build()
                        ),
                        new UnFlipAndHome(wrist, arm, claw),
                        new DelayedCommand(new SetJunction(lift, Junction.NONE), 200)
                ),
                new Release(claw),
                new WaitCommand(500),
                new Grab(claw)
        );
    }
}
