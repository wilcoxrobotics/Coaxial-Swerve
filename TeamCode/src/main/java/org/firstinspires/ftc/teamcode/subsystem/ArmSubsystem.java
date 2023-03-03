package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

@Config
public class ArmSubsystem extends SubsystemBase {

    private final ServoEx left;
    private final ServoEx right;
    public static double home = 0.33;
    public static double away = 0;
    public static double mid = 0.175;
    String mode="";
    public ArmSubsystem(ServoEx left, ServoEx right){
        this.left = left;
        this.right = right;
    }

    //write the function to rotate the servos between 0 and 1
    public void home(){
        left.setPosition(home);
        right.setPosition(home);
        mode="home";
    }

    public void away(){
        left.setPosition(away);
        right.setPosition(away);
        mode="away";
    }

    public void mid() {
        left.setPosition(mid);
        right.setPosition(mid);
    }

    public Command runHomeCommand() {
        return new InstantCommand(this::home,this);
    }

    public Command runAwayCommand() {
        return new InstantCommand(this::away,this);
    }

    public String getMode() {
        return mode;
    }
}
