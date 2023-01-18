package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.onbot.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.lib.onbot.GamepadWrapper;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.math.TimeManager;

@Config
@TeleOp(name="Austin Test Bot" )
public class TestTele extends LinearOpMode {

    private DcMotorWrapper linSlideLeft;
    private DcMotorWrapper linSlideRight;
    private TimeManager timeManager;

    private final int linSlideLowerBound = 0;
    private final int linSlideUpperBound = 2500;
    private final double linSlidePower = 1;

    private final double[] linSlidePositions = { 0.0, 0.45, 0.6, 0.88,};
    private int linSlidePosition = 0;
    private boolean linSlideUp = false;
    private double linSlideOffset = 0;

    public enum Junction {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    public static int LOW = -350;
    public static int MEDIUM = -850;
    public static int HIGH = -1750;
    public static int GROUND = -100;
    public static double speedL = 0.01;
    public static double speedR = 0.01;

    protected boolean clawState;

    public static double maxSpeedAxon = 0.14, maxSpeedTorque = 0.25, maxSpeedNormal = 0.11;
    public static double deadZone = 0.1;

    protected CRServoImplEx v4bMoveRight;
    public static double v4bMoveRightStart = -1, v4bMoveRightEnd = 0.2;

    protected CRServoImplEx v4bMoveLeft;
    public static double v4bMoveLeftStart = -1, v4bMoveLeftEnd = 0.2;

    protected CRServoImplEx clawPitch;
    public static double clawPitchStart = -1, clawPitchEnd = 0.5;

    protected CRServoImplEx clawOpen;
    public static double clawOpenStart = 0, clawOpenEnd = 1;

    protected CRServoImplEx clawRot;
    public static double clawRotStart = -.7, clawRotEnd = 1;

    public static double slowSpeedMultiplier = .3;

    protected DcMotorEx motorFrontLeft;
    protected DcMotorEx motorBackLeft;
    protected DcMotorEx motorFrontRight;
    protected DcMotorEx motorBackRight;



    protected DcMotorEx[] motors = {motorBackLeft, motorBackRight, motorFrontRight, motorFrontLeft};
    protected double x,y,rx;
    //    VoltageSensor voltageSensor;
//    Slides slides = new Slides(voltageSensor);
    static int target =0;
    static int target1 = 0;
    boolean slides = false;
    private GamepadWrapper gamepad1Wrapper;
    private GamepadWrapper gamepad2Wrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.gamepad2Wrapper = new GamepadWrapper()
                .setGamepad(gamepad2);

        GamepadEx gamepadLib = new GamepadEx(gamepad2);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.timeManager = new TimeManager()
                .setOpMode(this);
        DcMotorWrapper.setTimeManager(this.timeManager);

        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get ("rightRear");

        DcMotorEx v4bright = (DcMotorEx) hardwareMap.dcMotor.get( "rightSlide");
        DcMotorEx v4bleft = (DcMotorEx) hardwareMap.dcMotor.get("leftSlide");

        this.linSlideLeft = new DcMotorWrapper()
                .setDcMotor(hardwareMap.dcMotor.get("leftSlide"))
                .setLowerBound(this.linSlideLowerBound)
                .setUpperBound(-this.linSlideUpperBound)
                .setPower(-this.linSlidePower);
        this.linSlideRight = new DcMotorWrapper()
                .setDcMotor(hardwareMap.dcMotor.get("rightSlide"))
                .setLowerBound(this.linSlideLowerBound)
                .setUpperBound(this.linSlideUpperBound)
                .setPower(this.linSlidePower);

        DcMotor[] slides = {v4bleft, v4bright};
        for(DcMotor slide : slides) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        v4bright.setDirection(DcMotorSimple.Direction.REVERSE);
//        v4bright.setRunMode(Motor.RunMode.PositionControl);
//        v4bleft.setRunMode(Motor.RunMode.PositionControl);
//        double kPright = v4bright.getPositionCoefficient();
//        double kPleft = v4bleft.getPositionCoefficient();
//        v4bleft.setPositionTolerance(CONFIG.TOLERANCE);
//        v4bright.setPositionTolerance(CONFIG.TOLERANCE);


        v4bMoveLeft = (CRServoImplEx) hardwareMap.crservo.get("leftArm");
        v4bMoveRight = (CRServoImplEx) hardwareMap.crservo.get("rightArm");
        clawRot = (CRServoImplEx) hardwareMap.crservo.get("turret");
        clawPitch = (CRServoImplEx) hardwareMap.crservo.get("pitch");
        clawOpen = (CRServoImplEx) hardwareMap.crservo.get("claw");


        boolean rotButtonPrevState = gamepad1.right_bumper, clawButtonPrevState = gamepad1.left_bumper;
        //boolean rotButtonPrevState = gamepad1.
        boolean rotButtonState = true, clawButtonState = false;


        double slowModeValue = 1;
        boolean slowModeButtonPrevState = gamepad1.a;

        waitForStart();
        if (isStopRequested()) return;
        long time = System.currentTimeMillis();
        long prevTime = System.currentTimeMillis();
        while (opModeIsActive()) {

            if(gamepad1.a && !slowModeButtonPrevState)
                slowModeValue = slowSpeedMultiplier;
            else if(!gamepad1.a && slowModeButtonPrevState)
                slowModeValue = 1;

            slowModeButtonPrevState = gamepad1.a;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*slowModeValue,
                            -gamepad1.left_stick_x*slowModeValue,
                            gamepad1.right_stick_x*slowModeValue
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

//Servo code ----------------
//            double rotWriteState = 0, clawWriteState = 0, pitchWriteState = 0;

            if(gamepad1.right_bumper && !rotButtonPrevState){
                if(clawButtonState){
                    clawButtonState = !clawButtonState;
                    clawOpen.setPower(clawButtonState ? clawOpenStart : clawOpenEnd);
                }
                rotButtonState = !rotButtonState;
                clawRot.setPower(rotButtonState ? clawRotStart : clawRotEnd);
                clawPitch.setPower(rotButtonState ? clawPitchStart : clawPitchEnd);
                v4bMoveLeft.setPower(!rotButtonState ? v4bMoveLeftStart : v4bMoveLeftEnd);
                v4bMoveRight.setPower(!rotButtonState ? v4bMoveRightStart : v4bMoveRightEnd);
            }
//            else if()

            if(gamepad1.left_bumper && !clawButtonPrevState) {
                clawButtonState = !clawButtonState;
                clawOpen.setPower(clawButtonState ? clawOpenStart : clawOpenEnd);
            }

            //ARM

//            v4bMoveLeft.setPower(-gamepad2.right_stick_x);
//            v4bMoveRight.setPower(-gamepad2.right_stick_x);
            rotButtonPrevState = gamepad1.right_bumper;
            clawButtonPrevState = gamepad1.left_bumper;

//--------------------------------
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            //Rails
            double slidesValue = -gamepad1.right_trigger + gamepad1.left_trigger;
            v4bleft.setPower(slidesValue);
            v4bright.setPower(slidesValue);


            telemetry.addData("RRM Ticks", v4bright.getCurrentPosition());
            telemetry.addData("LRM Ticks", v4bleft.getCurrentPosition());
            telemetry.addData("Claw Power", clawOpen.getPower());
            dashboardTelemetry.addData("RRM Ticks", v4bright.getCurrentPosition());
            dashboardTelemetry.addData("LRM Ticks", v4bleft.getCurrentPosition());
            dashboardTelemetry.addData("Claw Power", clawOpen.getPower());
            dashboardTelemetry.update();
            telemetry.update();
            prevTime = System.currentTimeMillis();
        }


    }

    private void moveLinSlide(){
        this.linSlideLeft.setPosition(this.linSlidePositions[linSlidePosition] + linSlideOffset);
        this.linSlideRight.setPosition(this.linSlidePositions[linSlidePosition] + linSlideOffset);
    }


}