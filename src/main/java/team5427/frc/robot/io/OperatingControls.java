package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.commands.AllCommands;

public class OperatingControls {

  private CommandXboxController joy;

  private boolean coralMode = true;
  private boolean forward = true;

  public OperatingControls() {
    joy = new CommandXboxController(1);

    joy.a()
        .onTrue(
            new ConditionalCommand(
                AllCommands.scoreL1, AllCommands.scoreProcessor, () -> coralMode));
    joy.x().onTrue(AllCommands.scoreL2);
    joy.b().onTrue(AllCommands.scoreL3);
    joy.y().onTrue(new ConditionalCommand(
      new ConditionalCommand(AllCommands.scoreL4, AllCommands.scoreL4Inverse, () -> forward), 
      AllCommands.scoreBarge, 
      () -> coralMode));

    joy.leftTrigger()
        .whileTrue(
            new ConditionalCommand(
                AllCommands.intake, AllCommands.lowReefAlgaeIntake, () -> coralMode));
    joy.rightTrigger()
        .whileTrue(AllCommands.eject);

    joy.rightBumper().onTrue(
      new InstantCommand(() -> {forward = false;})
    );

    joy.leftBumper().onTrue(
      new InstantCommand(() -> {forward = true;})
    );
            // new ConditionalCommand(AllCommands.eject, AllCommands.ejectAlgae, () -> coralMode));
    // joy.povUp().onTrue(new InstantCommand(() -> {
    //   CascadeSubsystem.getInstance().getCurrentCommand().end(true);
    //
    // CascadeSubsystem.getInstance().setCascadeEncoderPosition(CascadeConstants.kCascadeMinimumHeight);
    // }));

    joy.povDown().onTrue(AllCommands.resetSubsystems);

    // joy.povUp().onTrue(AllCommands.climbStep);

    joy.povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  coralMode = true;
                }));

    joy.povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  coralMode = false;
                }));
  }
}
