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
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
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
@Autonomous(name = "Left Auto???")
public class AutonLeftCycle extends LinearOpMode {
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
        WAIT_FOR_PRELOAD,
        SLIDE,
        PRELOAD,
        DEPOSIT,
        WAIT_FOR_DEPOSIT,
        MOVE_TO_RETRIEVE,
        WAIT,
        RETRIEVE,
        WAIT_FOR_GRAB,
        BRUH,
        DEPOSIT_SLIDE,
        PARK,
        IDLE
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
        liftLeft  = hardwareMap.get(DcMotorSimple.class, "slideL");

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
            }
            //lift.update();
            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;


        currentState = DRIVE_PHASE.WAIT_FOR_PRELOAD;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case WAIT_FOR_PRELOAD:
                    wrist.home();
                    if(tagOfInterest == null) {
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(10)
                                .build());
                    }else if(tagOfInterest.id == 0) {

                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .forward(10)
                                        //.strafeRight(10)
                                .build());

                        lift.setHigh();

                    }else if(tagOfInterest.id == 1){
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .forward(10)
                                .build());
                    }else if(tagOfInterest.id == 2){
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .forward(10)

                                .build());
                    }
//                    delayedLift = true;
//                    drive.followTrajectoryAsync(drive.trajectoryBuilder(startPose)
//                            .lineTo(new Vector2d(29.16877, 0.158853) )
//                            .build());
//                    //slideSub.setPos(0.42);
//                    currentState = DRIVE_PHASE.SLIDE;

                    break;
                case SLIDE:
                    wrist.flip();
                    sleep(500);
                    currentState = DRIVE_PHASE.PARK;
                    break;
                case PARK:
                    if(!drive.isBusy()){
                        if(tagOfInterest == null) {
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(48, 0.2*reverse, Math.toRadians(180)*reverse))
                                    .build());
                        }else if(tagOfInterest.id == 0) {
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(45, 24.52*reverse, Math.toRadians(90)*reverse))
                                    .build());
                        }else if(tagOfInterest.id == 1){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(48, 0.2*reverse, Math.toRadians(180)*reverse))
                                    .build());
                        }else if(tagOfInterest.id == 2){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(47, -24.52*reverse, Math.toRadians(180)*reverse))
                                    .build());
                        }
                        currentState = DRIVE_PHASE.IDLE;
                    }
                    break;
//                case DEPOSIT_SLIDE:
//                    if (!drive.isBusy()) {
//                        slideSub.out();
//                        wait750.reset();
//                        currentState = DRIVE_PHASE.DEPOSIT;
//                    }
//                    break;
//                case DEPOSIT:
//                    if (!drive.isBusy() && wait750.seconds()>=0.4 && wait250.seconds() >= 0.75) {
//                        currentState = DRIVE_PHASE.WAIT_FOR_DEPOSIT;
//                        wait250.reset();
//                    }
//                    break;
//
//                case WAIT_FOR_DEPOSIT:
//                    if (wait250.seconds() >= 0) {
//                        clawSub.release();
//                        wait100.reset();
//                        currentState = DRIVE_PHASE.MOVE_TO_RETRIEVE;
//                    }
//                    break;
//                case MOVE_TO_RETRIEVE:
//                    if(wait100.seconds() >= 0.1){
//                        slideSub.in();
//                        liftSub.setJunction(pickupPosition);
//                        pickupPosition+=38;
//                        if(coneCounter <= 0){
//                            currentState = DRIVE_PHASE.PARK;
//                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                    .turn(Math.toRadians(-45)*reverse)
//                                    .build());
//                        }else{
//                            currentState = DRIVE_PHASE.WAIT;
//                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                    .forward(2)
//                                    .splineTo(new Vector2d(spline_x_pos, (spline_y_pos+10)*reverse), Math.toRadians(90)*reverse)
//                                    .forward(Math.abs(retrieve_y_pos-spline_y_pos-10))
//                                    .build());
//                            spline_x_pos += x_change*reverse;
//                            retrieve_y_pos -= y_change*reverse;
//                        }
//                        coneCounter--;
//
//                    }
//                    break;
//
//                case WAIT:
//                    if(!drive.isBusy()){
//                        liftTimer.reset();
//                        currentState=DRIVE_PHASE.RETRIEVE;
//                    }
//                    break;
//                case RETRIEVE:
//                    if (liftTimer.seconds()>=waitTime) {
//                        clawSub.grab();
//                        wait250.reset();
//                        currentState = DRIVE_PHASE.BRUH;
//                    }
//                    break;
//                case BRUH:
//                    if(wait250.seconds() >= 0.25){
//                        liftSub.setJunction(Junction.MEDIUM);
//                        liftTimer.reset();
//                        delayedLift = true;
//                        wait750.reset();
//                        currentState = DRIVE_PHASE.WAIT_FOR_GRAB;
//                        waitTime+=0.1;
//                    }
//                    break;
//                case WAIT_FOR_GRAB:
//                    if(wait750.seconds()>=0.5){
//                        delayedExtend = true;
//                        delayTimer.reset();
//                        currentState = DRIVE_PHASE.DEPOSIT_SLIDE;
//                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .back(Math.abs(spline_y_pos-deposit_y_pos-10))
//                                .splineTo(new Vector2d(deposit_x_pos, (deposit_y_pos*reverse)), Math.toRadians(303)*reverse)
//                                .build());
//                        deposit_x_pos += x_change*reverse;
//                        deposit_y_pos -= y_change*reverse;
//                    }
//                    break;
//                case PARK:
//                    if(!drive.isBusy()){
//                        if(tagOfInterest == null) {
//                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(48, 0.2*reverse, Math.toRadians(180)*reverse))
//                                    .build());
//                        }else if(tagOfInterest.id == 0) {
//                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(45, 24.52*reverse, Math.toRadians(90)*reverse))
//                                    .build());
//                        }else if(tagOfInterest.id == 1){
//                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(48, 0.2*reverse, Math.toRadians(180)*reverse))
//                                    .build());
//                        }else if(tagOfInterest.id == 2){
//                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(47, -24.52*reverse, Math.toRadians(180)*reverse))
//                                    .build());
//                        }
//                        currentState = DRIVE_PHASE.IDLE;
//                    }
//                    break;
//                case IDLE:
//                    deposit_y_pos = storedDepositY;
//                    deposit_x_pos = storedDepositX;
//                    spline_x_pos = storedSplineX;
//                    break;
                }
//                if (delayedExtend && delayTimer.seconds() >= 0.2) {
//                    delayedExtend = false;
//                    slideSub.setPos(0.42);
//                }

//                if (delayedLift && liftTimer.seconds() >= 0.75) {
//                    lift.setJunction(Junction.HIGH);
//                    delayedLift = false;
//                }

                drive.update();
                //lift.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("drive phase", currentState);
                telemetry.update();
            }
        }
        public void initCamera () {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
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

