package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.GuardedBy;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystem.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;

public class RobotHardware {

    public static boolean USING_IMU = true;

    private static RobotHardware instance = null;
    public static Pose error = new Pose(0, 0, 0);
    public static Pose targetPose = new Pose(0, 0, 0);
    public static boolean reached = false;
    public static boolean AUTO;
    public static Pose assumedPose = new Pose(0, 0, 0);
    public boolean enabled;
    private final Object imuLock = new Object();
    public static double imuOffset = 0.0;
    public Thread imuThread;

    private final Object IMULock = new Object();
    private double imuAngle = 0;
    private double imuAngleVelocity = 0;
    @GuardedBy("IMULock")
    private BNO055IMU imu;
    public Motor.Encoder parallelPod;
    public Motor.Encoder perpindicularPod;
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap) {
        synchronized (IMULock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }
        //FOR TWO WHEEL ODOMETRY
        parallelPod = new MotorEx(hardwareMap, "backLeftMotor").encoder;
        parallelPod.setDirection(Motor.Direction.REVERSE);
        //FOR TWO WHEEL ODOMETRY
        perpindicularPod = new MotorEx(hardwareMap, "backRightMotor").encoder;
        perpindicularPod.setDirection(Motor.Direction.REVERSE);
    }

    public void loop(Pose drive, SampleSwerveDrive drivetrain) {
        try {
            if (drive != null) {
                drivetrain.set(drive);
            }
            drivetrain.updateModules();
        } catch (Exception ignored) {}
    }

    public void read(SampleSwerveDrive drivetrain) {
        try {
            drivetrain.read();

        } catch (Exception ignored) {}
    }

    public void write(SampleSwerveDrive drivetrain) {
        try {
            drivetrain.write();

        } catch (Exception ignored) {}
    }
    public double getAngle() {
        return imuAngle - imuOffset;
    }

    public void clearBulkCache() {
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }
    public void startIMUThread(LinearOpMode opMode) {
        if (true) {
            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        imuAngle = imu.getAngularOrientation().firstAngle;
                    }
                }
            });
            imuThread.start();
        }
    }
    public void setIMUOffset(double offset) {
        imuOffset = offset;
    }
    public double getRawExternalHeading() {
        return imuAngle;
    }
    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return imuAngleVelocity;

    }

    public void reset() {
        try {
            parallelPod.reset();
            perpindicularPod.reset();
        } catch (Exception e) {}
        imuOffset = imu.getAngularOrientation().firstAngle;
    }
}
