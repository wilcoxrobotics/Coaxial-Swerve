package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.Junction;

public class BaseOpMode extends CommandOpMode {

    protected MotorEx leftBack, leftFront, rightBack, rightFront; //liftLeft;
    protected DcMotorSimple liftLeft;

    protected SimpleServo clawServo, armL, armR, wristServo;

    protected ClawSubsystem claw;
    protected DriveSubsystem drive;
    protected LiftSubsystem lift;
    protected ArmSubsystem arm;
    protected WristSubsystem wrist;
    boolean rightFrontB = false, rightBackB=false, leftFrontB=false, leftBackB=false;
    protected RevIMU imu;



    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        initHardware();
        setUp();

        drive = new DriveSubsystem(leftBack, leftFront, rightBack, rightFront);
        lift = new LiftSubsystem(liftLeft,  rightFront, gamepadEx2::getLeftY );
        claw = new ClawSubsystem(clawServo);
        arm = new ArmSubsystem(armL,armR);
        wrist = new WristSubsystem(wristServo);

        lift.setJunction(Junction.NONE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }

    protected void initHardware(){
        leftBack = new MotorEx(hardwareMap, "leftBack");
        leftFront = new MotorEx(hardwareMap, "leftFront");
        rightBack = new MotorEx(hardwareMap, "rightBack");
        rightFront = new MotorEx(hardwareMap, "rightFront");

        liftLeft  = hardwareMap.get(DcMotorSimple.class, "slideL");

        clawServo = new SimpleServo(hardwareMap, "claw", 0, 120);
        armL = new SimpleServo(hardwareMap, "armL", 0, 360);

        armR = new SimpleServo(hardwareMap, "armR", 0, 360);

        wristServo = new SimpleServo(hardwareMap, "wrist", 0, 180);

        imu = new RevIMU(hardwareMap);
        imu.init();

    }
    protected void setUp(){

//        leftFront.setInverted(true);
        leftBack.setInverted(true);
        wristServo.setInverted(true);

//        rightBack.setInverted(true);


        armR.setInverted(true);

        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightBack.resetEncoder();

        //liftLeft.setRunMode(Motor.RunMode.RawPower);
        //liftLeft.resetEncoder();
    }

    @Override
    public void run() {
        super.run();
        tad("leftBack Power", leftBack.motor.getPower());
        tad("leftFront Power", leftFront.motor.getPower());
        tad("rightBack Power", rightBack.motor.getPower());
        tad("rightFront Power", rightFront.motor.getPower());

        tad("liftLeft Power", liftLeft.getPower());
        tad("liftLeft Position", leftBack.getCurrentPosition());


        tad("Heading", imu.getHeading());

        tad("Current Junction", lift.getCurrentJunction());

        tad("output", lift.getOutput());
        tad("current target", lift.getCurrentTarget());
        tad("current junction", lift.getCurrentJunction());

        //wrist telemetry
        tad("wrist servo position", wristServo.getPosition());
        //arm telemetry
        tad("armL servo position", armL.getPosition());
        tad("armR servo position", armR.getPosition());
        //claw telemetry
        tad("claw servo position", clawServo.getPosition());

        telemetry.update();

    }

    // gamepad button 1 = gb1
    protected GamepadButton gb1(GamepadKeys.Button button){
        return gamepadEx1.getGamepadButton(button);
    }

    // gamepad button 2 = gb2
    protected GamepadButton gb2(GamepadKeys.Button button){
        return gamepadEx2.getGamepadButton(button);
    }

    protected void tad(String tag, Object data){
        telemetry.addData(tag, data);
    }
}
