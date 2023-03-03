package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
@Config
public class ClawSubsystem extends SubsystemBase {
    private final SimpleServo claw;

    public static double grabPosition = 0.4;
    public static double releasePosition = 0.65
            ;


    public ClawSubsystem(SimpleServo claw) {
        this.claw = claw;
    }

    public void grab(){
        claw.setPosition(grabPosition);
    }

    public void release(){
        claw.setPosition(releasePosition);
    }

    public Command runGrabCommand() {
        return new InstantCommand(this::grab, this);
    }

    public Command runReleaseCommand() {
        return new InstantCommand(this::release, this);
    }
}
