package org.firstinspires.ftc.teamcode.command.wrist;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.WristSubsystem;

public class Flip1 extends CommandBase {
    private final WristSubsystem wrist1;

    public Flip1(WristSubsystem wrist1){
        this.wrist1 = wrist1;
        addRequirements(wrist1);
    }

    @Override
    public void initialize() {
        wrist1.flip();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
