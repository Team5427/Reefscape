package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.commands.MoveElevatorTo;

public class OperatingControls {

    private CommandXboxController joy;

    public OperatingControls() {
        joy = new CommandXboxController(1);

        joy.povDown().onTrue(new MoveElevatorTo(CascadeConstants.kStowDistance));
        joy.a().onTrue(new MoveElevatorTo(CascadeConstants.kL1Distance));
        joy.x().onTrue(new MoveElevatorTo(CascadeConstants.kL2Distance));
        joy.b().onTrue(new MoveElevatorTo(CascadeConstants.kL3Distance));
        joy.y().onTrue(new MoveElevatorTo(CascadeConstants.kL4Distance));
    }
    
}
