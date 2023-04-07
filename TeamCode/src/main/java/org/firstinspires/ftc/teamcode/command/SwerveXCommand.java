package org.firstinspires.ftc.teamcode.command;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.SampleSwerveDrive;


public class SwerveXCommand extends CommandBase {
    private SampleSwerveDrive drivetrain;
    private ElapsedTime timeout;
    public SwerveXCommand(SampleSwerveDrive swerveDrivetrain) {
        drivetrain = swerveDrivetrain;
    }

    @Override
    public void execute() {
        if (timeout == null) timeout = new ElapsedTime();
        drivetrain.frontLeftModule.setTargetRotation(-PI / 4);
        drivetrain.frontRightModule.setTargetRotation(PI / 4);
        drivetrain.backRightModule.setTargetRotation(-PI / 4);
        drivetrain.backLeftModule.setTargetRotation(PI / 4);
        drivetrain.updateModules();
    }

    @Override
    public boolean isFinished() {
        return timeout.seconds() > 7;
    }
}