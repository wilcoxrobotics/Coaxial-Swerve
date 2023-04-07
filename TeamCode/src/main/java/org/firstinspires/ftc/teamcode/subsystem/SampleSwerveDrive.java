package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.*;
import static org.firstinspires.ftc.teamcode.util.RobotHardware.AUTO;

import androidx.annotation.GuardedBy;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.SwerveDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.util.geometry.MathUtils;

import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleSwerveDrive implements Drivetrain {
    public SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModule[] modules;
    private final VoltageSensor batteryVoltageSensor;
    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;
    private final double R;
    public static double frontLeftOffset = -2.65, frontRightOffset = -3.64, backLeftOffset = -1.91, backRightOffset = -1.95;

    public static double K_STATIC = 0.03;
    public static boolean maintainHeading = false;

    double[] ws = new double[4];
    double[] wa = new double[4];
    double max = 0.0;

    public static double minPow = 0.07;

    public SampleSwerveDrive(HardwareMap hardwareMap) {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        frontLeftModule = new SwerveModule(hardwareMap, "leftFrontMotor", "leftFrontServo", "leftFrontEncoder");
        backLeftModule = new SwerveModule(hardwareMap, "leftRearMotor", "leftRearServo", "leftRearEncoder");
        backRightModule = new SwerveModule(hardwareMap, "rightRearMotor", "rightRearServo", "rightRearEncoder");
        frontRightModule = new SwerveModule(hardwareMap, "rightFrontMotor", "rightFrontServo", "rightFrontEncoder");
        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = hypot(TRACK_WIDTH, WHEEL_BASE);

        PhotonCore.enable();
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
    }


    public void read() {
        for (SwerveModule module : modules) module.read();
    }



    @Override
    public void set(Pose pose) {
        set(pose, -1);
    }

    @Override
    public void set(Pose pose, double maxPower) {
        double x = pose.x, y = pose.y, head = pose.heading;
        if (maxPower != -1) {
            double r = Math.hypot(x, y);
            x = x / r * maxPower;
            y = y / r * maxPower;
        }

        double a = x - head * (WHEEL_BASE / R),
                b = x + head * (WHEEL_BASE / R),
                c = y - head * (TRACK_WIDTH / R),
                d = y + head * (TRACK_WIDTH / R);

        ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
        if (!maintainHeading) {
            wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
        }

        max = MathUtils.max(ws);
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.setMotorPower(Math.abs(ws[i]) + ((AUTO) ? minPow * Math.signum(ws[i]) : 0));
            m.setTargetRotation(MathUtils.norm(wa[i]));
        }
    }

    public void updateModules() {
        for (SwerveModule m : modules) m.update();
        SwerveModule.K_STATIC = K_STATIC;
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }



    public void setMode(DcMotor.RunMode runMode) {
        for (SwerveModule m : modules) m.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (SwerveModule m : modules) m.setZeroPowerBehavior(zeroPowerBehavior);

    }
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (SwerveModule m : modules) m.setPIDFCoefficients(runMode, compensatedCoefficients);
    }
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (SwerveModule m : modules) wheelPositions.add(m.getWheelPosition());
        return wheelPositions;
    }


    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (SwerveModule m : modules) wheelVelocities.add(m.getWheelVelocity());
        return wheelVelocities;
    }




    public List<Double> getModuleOrientations() {
        List<Double> moduleOrientations = new ArrayList<>();
        for (SwerveModule m : modules) moduleOrientations.add(m.getModuleRotation());

        return moduleOrientations;
    }

    public void setModuleOrientations(double v, double v1, double v2, double v3) {
        frontLeftModule.setTargetRotation(v);
        backLeftModule.setTargetRotation(v1);
        backRightModule.setTargetRotation(v2);
        frontLeftModule.setTargetRotation(v3);
    }

    public void setModuleVelocities(double v, double v1, double v2, double v3) {
        frontLeftModule.setServoPower(v);
        backLeftModule.setServoPower(v1);
        backRightModule.setServoPower(v2);
        frontRightModule.setServoPower(v3);

    }


}