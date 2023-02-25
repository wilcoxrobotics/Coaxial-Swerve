package org.firstinspires.ftc.teamcode.command.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class Home extends CommandBase {
    private final ArmSubsystem arm;

    public Home(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.home();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
