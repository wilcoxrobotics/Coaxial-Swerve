package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
import org.firstinspires.ftc.teamcode.command.arm.Mid;
import org.firstinspires.ftc.teamcode.command.wrist.Flip;
import org.firstinspires.ftc.teamcode.command.wrist.UnFlip;
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


    private enum DRIVE_PHASE {

        IDLE,
        TEST1,
        TEST2,
        TEST3,
        TEST4,
        TEST5,
        TEST6,
        TEST7
    }
    DRIVE_PHASE currentState = DRIVE_PHASE.IDLE;
    public static double initial_turn_angle = 0;

    protected MotorEx rightFront;
    protected DcMotorSimple liftLeft;
    protected GamepadEx gamepadEx1;

    public static int distance;

    private enum DRIVE_STATE {

        FIRST,
        SECOND,
        THIRD,
        FOURTH
    }
    DRIVE_STATE state = DRIVE_STATE.FIRST;

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

//        clawS.home();
        arm.mid();


        //INIT SHIT

        delayTimer.reset();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("LOOKING FOR CONE", true);
            telemetry.update();

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            boolean tagFound = false;


            if (currentDetections.size() != 0) {

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
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

        waitForStart();
        telemetry.addData("STARTED", true);
        telemetry.update();




        Pose2d startPose = new Pose2d(-36.00, -60, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence MainTrajectory = drive.trajectorySequenceBuilder(startPose)
                //drive forward
                .splineTo(new Vector2d(-36, -50), Math.toRadians(-90))
                //rotate 180
                .lineToLinearHeading(new Pose2d(-36, -50, Math.toRadians(90)))
                .addDisplacementMarker(() ->{
                    currentState = DRIVE_PHASE.TEST1;
                    //Run the Slides
                })
                //driving left to the pole
                .strafeTo(new Vector2d(-18, 0))
                .addDisplacementMarker(() ->{
                    //placing the cone
                    currentState = DRIVE_PHASE.TEST2;

                    //Run the Claw
                })
                .waitSeconds(1)
                .addDisplacementMarker(()->{
                    currentState = DRIVE_PHASE.TEST3;
                    //close slides
                })
                .waitSeconds(1)

                .build();


        TrajectorySequence ParkNull = drive.trajectorySequenceBuilder(MainTrajectory.end())
                .strafeRight(40)
                .build();

        TrajectorySequence Park1 = drive.trajectorySequenceBuilder(MainTrajectory.end())
                .strafeRight(5)
                .build();

        TrajectorySequence Park2 = drive.trajectorySequenceBuilder(MainTrajectory.end())
                .strafeRight(15)
                .build();

        TrajectorySequence Park3 = drive.trajectorySequenceBuilder(MainTrajectory.end())
                .strafeRight(25)
                .build();
        TrajectorySequence t = ParkNull;
        if(tagOfInterest == null)
            t = ParkNull;
        else if(tagOfInterest.id == 1)
            t = Park1;
        else if(tagOfInterest.id == 2)
            t = Park2;
        else if(tagOfInterest.id == 3)
            t = Park3;

            telemetry.addData("drive phase", currentState);
            telemetry.update();

            drive.followTrajectorySequenceAsync(MainTrajectory);
            

            telemetry.addData("drive phase", currentState);
            telemetry.update();



            while (opModeIsActive()) {
                drive.update();
                if (isStopRequested()) return;

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("drive phase", currentState);

                telemetry.update();
                
                
                if(!drive.isBusy()){
                    drive.followTrajectorySequenceAsync(t);
                }
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

