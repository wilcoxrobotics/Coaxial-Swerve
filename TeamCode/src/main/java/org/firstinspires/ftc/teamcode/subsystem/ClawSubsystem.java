package org.firstinspires.ftc.teamcode.subsystem;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.subsystem.constants.WristConstants;

import java.util.function.BooleanSupplier;

public class ClawSubsystem extends SubsystemBase {
    ServoEx wrist;
    static String mode="";
    public static boolean pos;
    public ClawSubsystem(ServoEx wrist) {
        this.wrist = wrist;
    }

    public void grab() {
        wrist.setPosition(WristConstants.home);
        mode= "home";
        pos = true;
    }
    public void release() {
        wrist.setPosition(WristConstants.flip);
        mode="flip";
        pos = false;
    }


    public static String getMode() {
        return mode;
    }
}
