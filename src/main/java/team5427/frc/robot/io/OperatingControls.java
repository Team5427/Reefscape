package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.commands.AllCommands;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem.GamePieceMode;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem.Level;

public class OperatingControls {

  private CommandXboxController joy;

  public OperatingControls() {
    joy = new CommandXboxController(1);

    joy.a()
        .onTrue(
            new ConditionalCommand(
                AllCommands.scoreL1,
                AllCommands.scoreProcessor,
                () -> ProngSubsystem.getGamePieceMode() == GamePieceMode.CORAL));
    joy.x().onTrue(AllCommands.scoreL2);
    joy.b()
        .onTrue(
            new ConditionalCommand(AllCommands.scoreL3, AllCommands.scoreL3Inverse, () -> true));
    joy.y()
        .onTrue(
            new ConditionalCommand(
                new ConditionalCommand(AllCommands.scoreL4, AllCommands.scoreL4Inverse, () -> true),
                AllCommands.scoreBarge,
                () -> ProngSubsystem.getGamePieceMode() == GamePieceMode.CORAL));

    joy.leftTrigger()
        .whileTrue(
            new ConditionalCommand(
                // AllCommands.intake,
                AllCommands.intakeRSC,
                new ConditionalCommand(
                    AllCommands.highReefAlgaeIntake,
                    new ConditionalCommand(
                        AllCommands.lowReefAlgaeIntake,
                        AllCommands.floorAlgaeIntake,
                        () -> ProngSubsystem.level == Level.LOW),
                    // AllCommands.lowReefAlgaeIntake,
                    () -> ProngSubsystem.level == Level.HIGH),
                () -> ProngSubsystem.getGamePieceMode() == GamePieceMode.CORAL));

    joy.rightTrigger()
        .whileTrue(
            new ConditionalCommand(
                AllCommands.eject,
                AllCommands.ejectAlgae,
                () -> ProngSubsystem.getGamePieceMode() == GamePieceMode.CORAL));

    joy.rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  // ProngSubsystem.level = ProngSubsystem.level == Level.FLOOR ? Level.LOW :
                  // Level.HIGH;
                  ProngSubsystem.level = Level.HIGH;
                }));

    joy.leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  // ProngSubsystem.level = ProngSubsystem.level == Level.FLOOR ? Level.LOW :
                  // Level.HIGH;
                  ProngSubsystem.level = Level.LOW;
                }));

    joy.povDown().whileTrue(AllCommands.resetSubsystems);

    // joy.povUp()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               ClimbStates step = ClimberSubsystem.getClimbState();
    //               Logger.recordOutput("Climber Step", step);
    //               switch (step) {
    //                 case kStow:
    //                   ClimberSubsystem.setClimbState(ClimbStates.kPrep);
    //                   step = ClimberSubsystem.getClimbState();
    //                 case kPrep:
    //                   CascadeSubsystem.getInstance()
    //                       .setCascadeSetpoint(CascadeConstants.kStowDistance);
    //                   CascadeSubsystem.getInstance()
    //                       .setPivotSetpoint(CascadeConstants.kClimbPrepRotation);
    //                   ClimberSubsystem.getInstance().setSetpoint(ClimbConstants.kPrepPosition);
    //                   LightsSubsystem.getInstance().setPattern(BlinkinConstants.kBlack);

    //                   break;
    //                 case kHook:
    //                   CascadeSubsystem.getInstance()
    //                       .setCascadeSetpoint(CascadeConstants.kStowDistance);
    //                   CascadeSubsystem.getInstance()
    //                       .setPivotSetpoint(CascadeConstants.kClimbPrepRotation);
    //                   ClimberSubsystem.getInstance().setSetpoint(ClimbConstants.kActivePosition);
    //                   LightsSubsystem.getInstance().setPattern(BlinkinConstants.kBlack);
    //                   break;
    //                 case kClimb:
    //                   CascadeSubsystem.getInstance()
    //                       .setCascadeSetpoint(CascadeConstants.kZeroPosition);
    //                   CascadeSubsystem.getInstance()
    //                       .setPivotSetpoint(CascadeConstants.kTempClimbRotation);
    //                   ClimberSubsystem.getInstance().setSetpoint(ClimbConstants.kActivePosition);
    //                   ProngSubsystem.getInstance()
    //                       .setWristSetpoint(ProngEffectorConstants.kClimbRotation);
    //                   LightsSubsystem.getInstance().setPattern(BlinkinConstants.kBlack);
    //                   break;
    //                 default:
    //                   break;
    //               }
    //             },
    //             ClimberSubsystem.getInstance(),
    //             ProngSubsystem.getInstance(),
    //             CascadeSubsystem.getInstance(),
    //             LightsSubsystem.getInstance()));
    joy.povUp().onTrue(AllCommands.climbStep);

    joy.povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  ProngSubsystem.setGamePieceMode(GamePieceMode.CORAL);
                }));

    joy.povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  ProngSubsystem.setGamePieceMode(GamePieceMode.ALGAE);
                }));

    // joy.leftStick().whileTrue(new HomeCascade());

    //     Trigger kFirstClimbStage =
    //         new Trigger(
    //             () -> {
    //               return Climb.step == Climb.kPrep;
    //             });
    //     kFirstClimbStage.whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               LightsSubsystem.getInstance().setPattern(BlinkinConstants.kHeartbeatGray);
    //             }));
    //     Trigger kSecondClimbStage =
    //         new Trigger(
    //             () -> {
    //               return Climb.step == Climb.kActivate;
    //             });
    //     kSecondClimbStage.whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               LightsSubsystem.getInstance().setPattern(BlinkinConstants.kHeartbeatRed);
    //             }));

    //     Trigger kThirdClimbStage =
    //         new Trigger(
    //             () -> {
    //               return Climb.step == Climb.kClimb;
    //             });
    //     kThirdClimbStage.whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               LightsSubsystem.getInstance().setPattern(BlinkinConstants.kHeartbeatWhite);
    //             }));
  }
}
