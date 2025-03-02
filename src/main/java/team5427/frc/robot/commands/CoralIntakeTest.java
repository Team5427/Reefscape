package team5427.frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.SuperStructureEnum.ProngEffectorStates;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

public class CoralIntakeTest extends Command {

  private ProngSubsystem prongEffectorSubsystem;

  public CoralIntakeTest() {
    prongEffectorSubsystem = ProngSubsystem.getInstance();
    addRequirements(prongEffectorSubsystem);
  }

  @Override
  public void initialize() {
    ProngSubsystem.state = ProngEffectorStates.CORAL_INTAKE;
  }

  @Override
  public void execute() {
    prongEffectorSubsystem.setRollerSetpoint(MetersPerSecond.of(1.0));
    prongEffectorSubsystem.setWristSetpoint(Rotation2d.fromDegrees(100));
  }

  @Override
  public boolean isFinished() {
    return ProngSubsystem.state == ProngEffectorStates.ALGAE_STOWED
        || ProngSubsystem.state == ProngEffectorStates.CORAL_STOWED;
  }

  @Override
  public void end(boolean interruped) {
    if (!interruped) {}
  }
}
