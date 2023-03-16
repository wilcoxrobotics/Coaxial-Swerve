package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class ColorSensorSubsystem {
    //color sensor subsystem with functions to check if it's looking at a cone(red or blue doesn't matter returns true either way)
    public NormalizedColorSensor colorSensor;
    public ColorSensorSubsystem(NormalizedColorSensor colorSensor){
        this.colorSensor = colorSensor;
        if (this.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)this.colorSensor).enableLight(true);
        }
    }

    public boolean checkCone() {
        //NormalizedRGBA colors = colorSensor.getNormalizedColors();
        //return colors.red >= 1 || colors.blue >= 1;
        return true;
    }
}
