package team5427.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team5427.frc.robot.Constants.BlinkinConstants;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.commands.cascade.Climb;
import team5427.frc.robot.commands.cascade.MoveElevatorTo;
import team5427.frc.robot.commands.homing.ResetSubsystems;
import team5427.frc.robot.commands.intake.Intake;
import team5427.frc.robot.commands.outtake.EjectGamePiece;
import team5427.frc.robot.commands.outtake.Score;
import team5427.frc.robot.subsystems.LightsSubsystem;

public class AllCommands {

  public static final Command moveElevatorL1 = new MoveElevatorTo(CascadeConstants.kL1Distance);
  public static final Command moveElevatorL2 = new MoveElevatorTo(CascadeConstants.kL2Distance);
  public static final Command moveElevatorL3 = new MoveElevatorTo(CascadeConstants.kL3Distance);
  public static final Command moveElevatorL4 = new MoveElevatorTo(CascadeConstants.kL4Distance);

  public static final Command scoreL1 = new Score(RobotConfigConstants.kScoreL1);
  public static final Command scoreL2 = new Score(RobotConfigConstants.kScoreL2);
  public static final Command scoreL3 = new Score(RobotConfigConstants.kScoreL3);
  public static final Command scoreL4 = new Score(RobotConfigConstants.kScoreL4);

  public static final Command scoreL3Inverse = new Score(RobotConfigConstants.kScoreL3Inverse);
  public static final Command scoreL4Inverse = new Score(RobotConfigConstants.kScoreL4Inverse);

  public static final Command scoreBarge = new Score(RobotConfigConstants.kScoreBarge);
  public static final Command scoreProcessor = new Score(RobotConfigConstants.kScoreProcessor);

  public static final Command climbStep = new Climb();

  public static final Command intake = new Intake(RobotConfigConstants.kCoralStationIntake);
  public static final Command lowReefAlgaeIntake =
      new Intake(RobotConfigConstants.kReefLowAlgaeIntake);
  public static final Command highReefAlgaeIntake =
      new Intake(RobotConfigConstants.kReefHighAlgaeIntake);
  public static final Command floorAlgaeIntake = new Intake(RobotConfigConstants.kAlgaeFloorIntake);
  public static final Command eject =
      new EjectGamePiece(true)
          .alongWith(
              new InstantCommand(
                  () -> {
                    LightsSubsystem.getInstance().setPattern(BlinkinConstants.kStrobeWhite);
                  }));
  // public static final Command floorIntake = new FloorIntake();
  public static final Command ejectAlgae =
      new EjectGamePiece(false)
          .alongWith(
              new InstantCommand(
                  () -> {
                    LightsSubsystem.getInstance().setPattern(BlinkinConstants.kStrobeBlue);
                  }));

  public static final Command resetSubsystems = new ResetSubsystems();

  // public static final Command switchToCoralMode =
  //     new InstantCommand(
  //         () -> {
  //           ProngSubsystem.gamePieceMode = GamePieceMode.CORAL;
  //           LightsSubsystem.getInstance().setPattern(BlinkinConstants.kWhite);
  //         });
  // public static final Command switchToAlgaeMode =
  //     new InstantCommand(
  //         () -> {
  //           ProngSubsystem.gamePieceMode = GamePieceMode.ALGAE;
  //           LightsSubsystem.getInstance().setPattern(BlinkinConstants.kBlue);
  //         });

  // public static final Command switchToHighLevel =
  //     new InstantCommand(
  //         () -> {
  //           // ProngSubsystem.level = ProngSubsystem.level == Level.FLOOR ? Level.LOW :
  // Level.HIGH;
  //           ProngSubsystem.level = Level.HIGH;
  //         });

  // public static final Command switchToLowLevel =
  //     new InstantCommand(
  //         () -> {
  //           // ProngSubsystem.level = ProngSubsystem.level == Level.HIGH ? Level.LOW :
  // Level.FLOOR;
  //           ProngSubsystem.level = Level.LOW;
  //         });
}
