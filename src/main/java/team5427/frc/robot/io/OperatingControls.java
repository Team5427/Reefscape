package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.commands.AllCommands;
import team5427.frc.robot.commands.homing.HomeCascade;
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
                () -> true));
    joy.y()
        .onTrue(
            new ConditionalCommand(
                new ConditionalCommand(
                    AllCommands.scoreL4,
                    AllCommands.scoreL4Inverse,
                    () -> true),
                AllCommands.scoreBarge,
                () -> ProngSubsystem.gamePieceMode == GamePieceMode.CORAL));

    joy.leftTrigger()
        .whileTrue(
            new ConditionalCommand(
                AllCommands.intake,
                new ConditionalCommand(
                    AllCommands.lowReefAlgaeIntake,
                    AllCommands.highReefAlgaeIntake,
                    () -> ProngSubsystem.direction == Direction.FORWARD
                ),
                () -> ProngSubsystem.gamePieceMode == GamePieceMode.CORAL));

    joy.rightTrigger().whileTrue(new ConditionalCommand(
        AllCommands.eject, 
        AllCommands.ejectAlgae, 
        () -> ProngSubsystem.gamePieceMode == GamePieceMode.CORAL));

    joy.rightBumper().onTrue(AllCommands.switchToInverse);

    joy.leftBumper().onTrue(AllCommands.switchToDirect);

    joy.povDown().onTrue(AllCommands.resetSubsystems);

    joy.povUp().onTrue(AllCommands.climbStep);

    joy.povRight().onTrue(AllCommands.switchToCoralMode);

    joy.povLeft().onTrue(AllCommands.switchToAlgaeMode);

    joy.leftStick().whileTrue(new HomeCascade());
  }
}
