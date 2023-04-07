package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.command.PositionCommand;
import org.firstinspires.ftc.teamcode.command.SwerveXCommand;
import org.firstinspires.ftc.teamcode.subsystem.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.subsystem.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystem.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
@Autonomous(name = "Example")
@Config
public class ExampleAuto extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private SampleSwerveDrive drivetrain;
    private TwoWheelLocalizer localizer;
    private ElapsedTime timer;
    private double loopTime;
    private double endtime = 0;
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        RobotHardware.USING_IMU = true;
        drivetrain = new SampleSwerveDrive(hardwareMap);
        robot.init(hardwareMap);
        localizer = new TwoWheelLocalizer(robot);
        robot.enabled = true;
        RobotHardware.AUTO = true;

        while (!isStarted()) {
            robot.read(drivetrain);
            for (SwerveModule module : drivetrain.modules) {
                module.setTargetRotation(Math.PI / 2);
            }
            drivetrain.updateModules();

            telemetry.addLine("EXAMPLE AUTONOMOUS WITH SWERVE CODE");
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain);
        }
        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //goto Position
                        new PositionCommand(drivetrain, localizer, new Pose(0 , 10,  0.235), 250, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        //reset modules
                        new SwerveXCommand(drivetrain)
                )
        );

        while (opModeIsActive()) {
            if (timer == null) {
                timer = new ElapsedTime();
            }
            robot.read(drivetrain);
            CommandScheduler.getInstance().run();
            robot.loop(null, drivetrain);
            localizer.periodic();
            telemetry.addData("time", endtime);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            robot.write(drivetrain);
            robot.clearBulkCache();
        }
    }
}
