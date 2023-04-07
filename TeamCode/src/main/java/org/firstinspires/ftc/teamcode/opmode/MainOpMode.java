package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;

@Config
@TeleOp(name = "OpMode")
public class MainOpMode extends CommandOpMode {
    private ElapsedTime timer;
    private ElapsedTime timer2;
    private double loopTime = 0;
    private SampleSwerveDrive drivetrain;
    RobotHardware robot = new RobotHardware();
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RobotHardware.USING_IMU = true;

        robot.init(hardwareMap);

        drivetrain = new SampleSwerveDrive(hardwareMap);

        robot.enabled = true;
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            try {
                robot.reset();
                robot.startIMUThread(this);
            } catch (Exception e) {
            }
            RobotHardware.imuOffset = -Math.PI / 2;
        }
        if (gamepad1.right_stick_button && RobotHardware.USING_IMU) {
            RobotHardware.imuOffset = robot.getAngle() + Math.PI;
        }
        double extended = (Math.abs(gamepad1.right_stick_x) > 0.98) ? 1 : 0.5;
        SampleSwerveDrive.maintainHeading = (Math.abs(gamepad1.left_stick_x) < 0.02 & Math.abs(gamepad1.left_stick_y) < 0.02 & Math.abs(gamepad1.right_stick_x) < 0.02);
        double rotationAmount = (RobotHardware.USING_IMU) ? robot.getAngle() - RobotHardware.imuOffset : 0;
        Pose drive = new Pose(
                new Point((Math.pow(Math.abs(gamepad1.left_stick_y) > 0.01 ? gamepad1.left_stick_y : 0, 3)),
                        (-Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.01 ? gamepad1.left_stick_x : 0), 3))).rotate(rotationAmount),
                -(Math.pow(-gamepad1.right_stick_x, 3)) * extended
        );
        robot.loop(drive, drivetrain);
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        robot.clearBulkCache();
    }
}
