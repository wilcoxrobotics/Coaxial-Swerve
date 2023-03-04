//package org.firstinspires.ftc.teamcode.subsystem;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.hardware.ServoEx;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//
//
//
//public class ClawColorSubsystem extends SubsystemBase {
//    private final ColorSensor colorSensor;
//    //claw
//    private final ServoEx claw;
//
//    public ClawColorSubsystem(ColorSensor colorSensor, ServoEx claw) {
//        this.claw = claw;
//        this.colorSensor = colorSensor;
//    }
//
//    public void grab(){
//        claw.setPosition(grabPosition);
//    }
//
//    publivoid release(){
//        claw.setPosition(releasePosition);
//    }
//
//    public void getColor() {
//        colorSensor.red();
//        colorSensor.green();
//        colorSensor.blue();
//    }
//}
