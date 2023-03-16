package org.firstinspires.ftc.teamcode.command.group;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.teamcode.command.arm.Away;
import org.firstinspires.ftc.teamcode.command.claw.Grab;
import org.firstinspires.ftc.teamcode.command.claw.Release;
import org.firstinspires.ftc.teamcode.command.lift.SetJunction;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.util.Junction;
@Config
public class AutoGrabRotateMoveLift extends SequentialCommandGroup {
    public static int forward=23;
    public AutoGrabRotateMoveLift(ColorSensorSubsystem colorSensor, SampleMecanumDrive autoDrive, ArmSubsystem arm, WristSubsystem wrist, ClawSubsystem claw, LiftSubsystem lift, Junction junction) {
        addCommands(
                new Release(claw),
                new WaitCommand(400),
                new Release(claw),
                new ParallelCommandGroup(
                        new GrabRotateLiftRelease(arm, lift, wrist, claw, junction),
                        new FollowTrajectorySequenceCommand(autoDrive, autoDrive.trajectoryBuilder(new Pose2d(0, 0, 0))
                                .forward(forward)
                                .build())

                ), new DelayedCommand(new Grab(claw), 500)


//                new ParallelCommandGroup(new FollowTrajectorySequenceCommand(autoDrive, autoDrive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                        .forward(30)
//                        .build()
//                ), new GrabRotateLift( arm, lift, wrist, claw, junction))
        );
        addRequirements( arm, wrist, lift, claw);
    }

}
