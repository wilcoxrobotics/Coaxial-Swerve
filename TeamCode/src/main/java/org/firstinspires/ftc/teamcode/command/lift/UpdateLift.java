package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class UpdateLift extends CommandBase {
    private final LiftSubsystem lift;

    public UpdateLift(LiftSubsystem lift) {
        this.lift = lift;

        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.update();
    }
}
