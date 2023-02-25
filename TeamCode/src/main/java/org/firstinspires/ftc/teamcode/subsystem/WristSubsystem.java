package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.subsystem.constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    ServoEx wrist;
    static String mode="";
    public WristSubsystem(ServoEx wrist) {
        this.wrist = wrist;
    }

    public void home() {
        wrist.setPosition(WristConstants.home);
        mode= "home";
    }


    public void flip() {
        wrist.setPosition(WristConstants.flip);
        mode="flip";
    }

    public Command runHomeCommand() {
        return new InstantCommand(this::home, this);
    }

    public Command runFlipCommand() {
        return new InstantCommand(this::flip, this);
    }

    public static String getMode() {
        return mode;
    }
}