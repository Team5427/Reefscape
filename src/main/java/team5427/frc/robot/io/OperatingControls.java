package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.commands.AllCommands;
import team5427.frc.robot.commands.HomeCascade;

import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem.Direction;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem.GamePieceMode;

public class OperatingControls {

  private CommandXboxController joy;

  public OperatingControls() {
    joy = new CommandXboxController(1);

    joy.a()
        .onTrue(
            new ConditionalCommand(
                AllCommands.scoreL1, 
                AllCommands.scoreProcessor, 
                () -> ProngSubsystem.gamePieceMode == GamePieceMode.CORAL));
    joy.x().onTrue(AllCommands.scoreL2);
    joy.b()
        .onTrue(
            new ConditionalCommand(
              AllCommands.scoreL3, 
              AllCommands.scoreL3Inverse, 
              () -> ProngSubsystem.direction == Direction.FORWARD));
    joy.y()
        .onTrue(
            new ConditionalCommand(
                new ConditionalCommand(
                    AllCommands.scoreL4, 
                    AllCommands.scoreL4Inverse, 
                    () -> ProngSubsystem.direction == Direction.FORWARD),
                AllCommands.scoreBarge,
                () -> ProngSubsystem.gamePieceMode == GamePieceMode.CORAL));

    joy.leftTrigger()
        .whileTrue(
            new ConditionalCommand(
                AllCommands.intake, 
                AllCommands.lowReefAlgaeIntake, 
                () -> ProngSubsystem.gamePieceMode == GamePieceMode.CORAL));

    joy.rightTrigger().whileTrue(AllCommands.eject);

    joy.rightBumper().onTrue(AllCommands.switchToInverse);

    joy.leftBumper().onTrue(AllCommands.switchToDirect);

    joy.povDown().onTrue(AllCommands.resetSubsystems);

    joy.povUp().onTrue(AllCommands.climbStep);

    joy.povRight().onTrue(AllCommands.switchToCoralMode);

    joy.povLeft().onTrue(AllCommands.switchToAlgaeMode);

    joy.leftStick().whileTrue(new HomeCascade());
  }
}
