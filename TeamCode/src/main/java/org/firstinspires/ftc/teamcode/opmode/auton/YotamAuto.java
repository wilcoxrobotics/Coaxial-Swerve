package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous
public class YotamAuto extends CommandOpMode {

    int reverse = 1;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    // UNITS ARE METERS
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;

    protected MotorEx rightFront;
    protected DcMotorSimple liftLeft;
    protected GamepadEx gamepadEx1;

    boolean delayedExtend = false;
    boolean delayedLift = false;
    double waitTime = 0.05;
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime wait100 = new ElapsedTime();
    ElapsedTime wait250 = new ElapsedTime();
    ElapsedTime wait750 = new ElapsedTime();

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));


    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);

        DcMotorSimple liftMotor = hardwareMap.get(DcMotorSimple.class, "slideL");
        rightFront = new MotorEx(hardwareMap, "rightFront");

        SimpleServo clawServo = new SimpleServo(hardwareMap, "claw", 0, 360);
        SimpleServo armL = new SimpleServo(hardwareMap, "armL", 0, 360);

        SimpleServo armR = new SimpleServo(hardwareMap, "armR", 0, 360);

        SimpleServo wristServo = new SimpleServo(hardwareMap, "wrist", 0, 180);
        liftLeft = hardwareMap.get(DcMotorSimple.class, "slideL");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSubsystem arm = new ArmSubsystem(armL, armR);
        ClawSubsystem claw = new ClawSubsystem(clawServo);
        WristSubsystem wrist = new WristSubsystem(wristServo);
        LiftSubsystem lift = new LiftSubsystem(liftLeft, rightFront, gamepadEx1::getLeftY);
        initCamera();

        drive.setPoseEstimate(startPose);
        //wrist.flip();
        sleep(1000);
        arm.away();
        sleep(1000);
        claw.release();
        sleep(500);
        arm.mid();
        sleep(500);
        //lift.setJunction(Junction.NONE);
        delayTimer.reset();

        // scan april tag
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addData("Tag:", tagOfInterest.id);
                    telemetry.addData("initialized", true);
                } else {
                    telemetry.addData("Not Tag Found", false);
                }
            }
            //lift.update();
            telemetry.update();
        }


        TrajectorySequence preload = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
                .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
                .build();

        TrajectorySequence cycleCone = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
                .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
                .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
                .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
                .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
                .build();

        TrajectorySequence parkNull = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
                .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
                .build();

        //
        schedule(
                new SequentialCommandGroup(
                    new FollowTrajectorySequenceCommand(drive, preload)
                )
        );
    }

    public void initCamera () {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720,  OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
}
