package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

@Config
public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;

    public static double slowModeFactor = 3;
    public static double slowRotScale = .75;
    public DriveSubsystem(MotorEx leftBack, MotorEx leftFront, MotorEx rightBack, MotorEx rightFront){
        drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
    }


    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle){
        drive.driveFieldCentric(-strafeSpeed, -forwardSpeed, -turnSpeed, gyroAngle);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed){
        drive.driveRobotCentric(-strafeSpeed, -forwardSpeed, -turnSpeed*slowRotScale);
    }

    public void driveRobotCentricSlowMode(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(-strafeSpeed / slowModeFactor,
                -forwardSpeed / slowModeFactor,
                -turnSpeed / slowModeFactor);
    }

//    public Command runRobotCentricCommand(double strafeSpeed, double forwardSpeed, double turnSpeed) {
//        return new RunCommand(() -> drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed), this);
//    }
}
