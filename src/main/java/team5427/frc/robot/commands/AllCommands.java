package team5427.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.ProngEffectorConstants;
import team5427.frc.robot.Constants.ClimbConstants;
import team5427.frc.robot.Constants.ScoringConstants;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.Climb.ClimberSubsystem;

public class AllCommands {

  public static final Command moveElevatorL1 = new MoveElevatorTo(CascadeConstants.kL1Distance);
  public static final Command moveElevatorL2 = new MoveElevatorTo(CascadeConstants.kL2Distance);
  public static final Command moveElevatorL3 = new MoveElevatorTo(CascadeConstants.kL3Distance);
  public static final Command moveElevatorL4 = new MoveElevatorTo(CascadeConstants.kL4Distance);

  public static final Command scoreL1 = new Score(ScoringConstants.kScoreL1);
  public static final Command scoreL2 = new Score(ScoringConstants.kScoreL2);
  public static final Command scoreL3 = new Score(ScoringConstants.kScoreL3);

  public static final Command climbStep = new Climb();

  public static final Command intake = new CoralIntakeTest();
  public static final Command eject = new EjectCoral();
  public static final Command floorIntake = new FloorIntake();
  public static final Command ejectAlgae = new EjectAlgae();

  public static final Command resetSubsystems =
      new InstantCommand(
          () -> {
            CascadeSubsystem.getInstance().setPivotSetpoint(CascadeConstants.kStowRotation);
            CascadeSubsystem.getInstance().setCascadeSetpoint(CascadeConstants.kStowDistance);
            ClimberSubsystem.getInstance().setSetpoint(ClimbConstants.kStowPosition);
            Climb.step = Climb.kReset;
          });
}
