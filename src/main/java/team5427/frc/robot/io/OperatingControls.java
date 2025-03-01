package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.ClimbConstants;
import team5427.frc.robot.commands.MoveElevatorTo;
import team5427.frc.robot.commands.Climb;
import team5427.frc.robot.commands.AllCommands;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.Climb.ClimberSubsystem;

public class OperatingControls {

  private CommandXboxController joy;

  public OperatingControls() {
    joy = new CommandXboxController(1);

    joy.a().onTrue(AllCommands.moveElevatorL1);
    joy.x().onTrue(AllCommands.moveElevatorL2);
    joy.b().onTrue(AllCommands.moveElevatorL3);
    joy.y().onTrue(AllCommands.moveElevatorL4);
    // joy.povUp().onTrue(new InstantCommand(() -> {
    //   CascadeSubsystem.getInstance().getCurrentCommand().end(true);
    //   CascadeSubsystem.getInstance().setCascadeEncoderPosition(CascadeConstants.kCascadeMinimumHeight);
    // }));

    joy.povDown().onTrue(AllCommands.resetSubsystems);

    joy.povUp().onTrue(AllCommands.climbStep);
  }
}