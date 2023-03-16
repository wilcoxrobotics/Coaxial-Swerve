package org.firstinspires.ftc.teamcode.subsystem;

import android.location.GnssMeasurement;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.DoubleSupplier;

@Config
public class ArmSubsystem extends SubsystemBase {

    private final ServoEx left;
    private final ServoEx right;
    public static double home = 0.78;
    public static double away = 0.32;
    public static double down = 0.1;
    public static double scaleFactor =.25;
    public static double mid = 0.55;
    public double valueRight = .32;
    public double valueLeft = .32;

    public DoubleSupplier adouble;
    String mode="";
    public ArmSubsystem(ServoEx left, ServoEx right){
        this.left = left;
        this.right = right;
        adouble = () -> 0;
    }

    public ArmSubsystem(ServoEx left, ServoEx right, DoubleSupplier adouble){
        this.left = left;
        this.right = right;
        this.adouble = adouble;
    }

    //write the function to rotate the servos between 0 and 1
    public void home(){
        valueRight = home;
        valueLeft = home;
//        left.setPosition(home);
//        right.setPosition(home);
        mode="home";
    }

    public void away(){
        valueRight = away;
        valueLeft = away;
//        left.setPosition(away);
//        right.setPosition(away);
        mode="away";
    }

    public void mid() {
        valueLeft = mid;
        valueRight = mid;
//        left.setPosition(mid);
//        right.setPosition(mid);
    }

    public void deposit(){
        valueLeft=down;
        valueRight=down;
    }

    @Override
    public void periodic(){
        left.setPosition(valueLeft+(scaleFactor* adouble.getAsDouble()));
        right.setPosition(valueRight+(scaleFactor* adouble.getAsDouble()));
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
