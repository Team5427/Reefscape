package team5427.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem.Direction;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem.GamePieceMode;

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
  public static final Command eject = new EjectGamePiece(true);
  public static final Command floorIntake = new FloorIntake();
  public static final Command ejectAlgae = new EjectGamePiece(false);

  public static final Command resetSubsystems = new ResetSubsystems();

  public static final Command switchToCoralMode = new InstantCommand(() -> {ProngSubsystem.gamePieceMode = GamePieceMode.CORAL;});
  public static final Command switchToAlgaeMode = new InstantCommand(() -> {ProngSubsystem.gamePieceMode = GamePieceMode.ALGAE;});

  public static final Command switchToDirect = new InstantCommand(() -> {ProngSubsystem.direction = Direction.FORWARD;});
  public static final Command switchToInverse = new InstantCommand(() -> {ProngSubsystem.direction = Direction.BACKWARD;});

}