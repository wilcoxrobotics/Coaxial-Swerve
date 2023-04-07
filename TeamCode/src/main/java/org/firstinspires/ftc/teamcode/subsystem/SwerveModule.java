package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.DriveConstants;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.CRServoProfiler;
import org.firstinspires.ftc.teamcode.util.Encoder;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

@Config
public class SwerveModule {
//    public static PIDCoefficients MODULE_PID = new PIDCoefficients(0.6, 0, 0.03);
//
//    public static double K_STATIC = 0.2, K_MOTOR = 0;

    public static CRServoProfiler.Constraints SERVO_CONSTRAINTS = new CRServoProfiler.Constraints(1, 1000, 2);

    public static double MAX_SERVO = 1, MAX_MOTOR = 1;

    //EXPERIMENTAL FEATURES
    public static boolean WAIT_FOR_TARGET = false;

    public static double ALLOWED_COS_ERROR = Math.toRadians(2);

    public static double ALLOWED_BB_ERROR = Math.toRadians(5);

    public static boolean MOTOR_FLIPPING = true;

    public static double FLIP_BIAS = Math.toRadians(0);
    private double position = 0.0;

    private DcMotorEx motor;
    private CRServo servo;
    private AbsoluteAnalogEncoder encoder;
    private PIDFController rotationController;
    private CRServoProfiler rotationProfiler;

    public boolean wheelFlipped = false;
    private double target = 0.0;


    public static double P = 0.6, I = 0, D = 0.1;
    public static double K_STATIC = 0.04;

    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);

        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
        //servo.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = e;
        e.setInverted(true);
        rotationController = new PIDFController(P, I, D, 0);
        rotationProfiler = new CRServoProfiler(servo, SERVO_CONSTRAINTS);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public SwerveModule(HardwareMap hardwareMap, String mName, String sName, String eName) {
        this(hardwareMap.get(DcMotorEx.class, mName),
                hardwareMap.get(CRServo.class, sName),
                new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, eName)));
    }


    public void update() {
        rotationController.setPIDF(P, I, D, 0);
        double target = getTargetRotation(), current = getModuleRotation();

        double error = normalizeRadians(target - current);
        if (MOTOR_FLIPPING && Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        error = normalizeRadians(target - current);

        double power = Range.clip(rotationController.calculate(0, error), -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(power)) power = 0;
        servo.setPower(power + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power));
    }

    public double getTargetRotation() {
        return normalizeRadians(target - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(position - Math.PI);
    }
    public void setMotorPower(double power) {
        if (wheelFlipped) power *= -1;
        lastMotorPower = power;
        motor.setPower(power);
    }
    public int flipModifier() {
        return wheelFlipped ? -1 : 1;
    }

    public void setTargetRotation(double target) {
        this.target = normalizeRadians(target);
    }

    public double getWheelPosition() {
        return DriveConstants.encoderTicksToInches(motor.getCurrentPosition());
    }

    public double getWheelVelocity() {
        return DriveConstants.encoderTicksToInches(motor.getVelocity());
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(runMode, coefficients);
    }
    double lastMotorPower = 0;

    public void read() {
        position = encoder.getCurrentPosition();
    }
    public static double MIN_MOTOR_TO_TURN = 0.05;

    public SwerveModuleState asState(){
        return new SwerveModuleState(this);
    }

    public double getServoPower() {
        return servo.getPower();
    }

    public static class SwerveModuleState {
        public SwerveModule module;
        public double wheelPos, podRot;
        public SwerveModuleState(SwerveModule s){
            module = s;
            wheelPos = 0;
            podRot = 0;
        }

        public SwerveModuleState update(){
            return setState(-module.getWheelPosition(), module.getModuleRotation());
        }
        public SwerveModuleState setState(double wheel, double pod){
            wheelPos = wheel;
            podRot = pod;
            return this;
        }
        //TODO add averaging for podrots based off of past values
        public Vector2d calculateDelta(){
            double oldWheel = wheelPos;
            update();
            return Vector2d.polar(wheelPos-oldWheel, podRot);
        }
    }
}