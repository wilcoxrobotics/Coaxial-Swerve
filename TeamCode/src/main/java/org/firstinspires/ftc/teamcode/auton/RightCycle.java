package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

public class RightCycle extends LinearOpMode {

    //create a 1+5 autonamous program that starts on the right side of the field

    int reverse = -1;
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
    private MotorEx liftMotor;
    private SimpleServo claw, wrist, armL, armR;

    private SampleMecanumDrive drive;
    private ClawSubsystem clawSub;
    private LiftSubsystem lift;
    private WristSubsystem wristSub;
    private ArmSubsystem armSub;
    @Override
    public void runOpMode() throws InterruptedException {
        liftMotor = new MotorEx(hardwareMap, "liftMotor");
        claw = new SimpleServo(hardwareMap, "claw", 0, 120);
        wrist = new SimpleServo(hardwareMap, "wrist", 0, 180);
        armL = new SimpleServo(hardwareMap, "armL", 0, 360);
        armR = new SimpleServo(hardwareMap, "armR",0,360);
    }
}
