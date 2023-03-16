package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "AustinCarriesSoftware")
public class Auton1p5 extends LinearOpMode {
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
    boolean delayedExtend = false;
    boolean delayedLift = false;
    double waitTime = 0.05;
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime wait100 = new ElapsedTime();
    ElapsedTime wait250 = new ElapsedTime();
    ElapsedTime wait750 = new ElapsedTime();

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

    private enum DRIVE_PHASE {

        IDLE,
        TEST1,
        TEST2,
        TEST3,
        TEST4
    }
    DRIVE_PHASE currentState = DRIVE_PHASE.IDLE;
    public static double initial_turn_angle = 0;

    protected MotorEx rightFront;
    protected DcMotorSimple liftLeft;
    protected GamepadEx gamepadEx1;

    public static int distance;

    @Override
    public void runOpMode() throws InterruptedException {
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
                //lift.update();
                telemetry.update();

            }
        }
        waitForStart();

        TrajectorySequence MainTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
                .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
                .build();

//            TrajectorySequence CycleCone = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
//                    .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
//                    .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
//                    .build();
//
//            TrajectorySequence Park1 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
//                    .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
//                    .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
//                    .build();
//
//            TrajectorySequence Park2 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
//                    .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
//                    .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
//                    .build();
//
//            TrajectorySequence Park3 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
//                    .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
//                    .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
//                    .build();
//
//            TrajectorySequence ParkNull = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.73, Math.toRadians(90.00)))
//                    .splineTo(new Vector2d(-36.31, -11.74), Math.toRadians(90.37))
//                    .splineTo(new Vector2d(-30.99, -7.20), Math.toRadians(40.46))
//                    .build();

        currentState = DRIVE_PHASE.TEST1;
        telemetry.addData("drive phase", currentState);
        telemetry.update();

        drive.followTrajectorySequenceAsync(MainTrajectory);
//        for(int i = 0; i < 5; i++){
//            drive.followTrajectorySequenceAsync(CycleCone);
//        }
//
//        if(tagOfInterest == null)
//            drive.followTrajectorySequenceAsync(ParkNull);
//
//        else if(tagOfInterest.id == 1)
//            drive.followTrajectorySequenceAsync(Park1);
//        else if(tagOfInterest.id == 2)
//            drive.followTrajectorySequenceAsync(Park2);
//        else if(tagOfInterest.id == 3)
//            drive.followTrajectorySequenceAsync(Park3);
        currentState = DRIVE_PHASE.TEST2;
        telemetry.addData("drive phase", currentState);
        telemetry.update();

        while (opModeIsActive()) {
            drive.update();
            if (isStopRequested()) return;

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            currentState = DRIVE_PHASE.TEST3;
            telemetry.addData("drive phase", currentState);

            telemetry.update();

        }
        currentState = DRIVE_PHASE.TEST4;


    }
    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
}

