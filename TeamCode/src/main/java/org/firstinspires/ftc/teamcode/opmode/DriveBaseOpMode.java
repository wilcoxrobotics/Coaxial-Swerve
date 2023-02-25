package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class DriveBaseOpMode extends CommandOpMode {
    protected MotorEx fL, fR, bL, bR;
    protected DriveSubsystem drive;
    protected GamepadEx gamepadEx1;
    protected RevIMU imu;
    @Override
    public void initialize() {
       gamepadEx1 = new GamepadEx(gamepad1);
       initHardware();
       setUpHardwareDevices();
       imu = new RevIMU(hardwareMap);
       imu.init();
       drive = new DriveSubsystem(fL, fR, bL, bR);

       telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
       telemetry.addData("Mode", "Done initializing");
       telemetry.update();
    }



    protected void initHardware() {
        fL = new MotorEx(hardwareMap, "leftFront");
        fR = new MotorEx(hardwareMap, "rightFront");
        bL = new MotorEx(hardwareMap, "leftBack");
        bR = new MotorEx(hardwareMap, "rightBack");
    }

    @Override
    public void run() {
        super.run();
        tad("leftFront Power", round(fL.motor.getPower()));
        tad("leftBack Power", round(bL.motor.getPower()));
        tad("rightFront Power", round(fR.motor.getPower()));
        tad("rightBack Power", round(bR.motor.getPower()));
        telemetry.update();
    }

    protected void setUpHardwareDevices() {
        fR.setInverted(false);
        bL.setInverted(false);
        bR.setInverted(true);
        fL.setInverted(true);
    }


    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        return new BigDecimal(Double.toString(value)).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }

    private static double round(double value) {
        return round(value, 4);
    }

    // gamepad button 1 = gb1
    protected GamepadButton gb1(GamepadKeys.Button button){
        return gamepadEx1.getGamepadButton(button);
    }



    // telemetry add data = tad
    protected void tad(String caption, Object value){
        telemetry.addData(caption, value);
    }
}
