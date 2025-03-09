package team5427.frc.robot.commands.cascade;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.SuperStructureEnum.CascadeStates;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;

public class CascadeTest extends Command {

  private CascadeSubsystem cascadeSubsystem;

  public CascadeTest() {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    addRequirements(cascadeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    CascadeSubsystem.state = CascadeStates.GROUND_INTAKE;
    cascadeSubsystem.setCascadeSetpoint(Meter.of(0.4));
    cascadeSubsystem.setPivotSetpoint(Rotation2d.fromDegrees(13));
  }

  // @Override
  // public boolean isFinished() {
  //     return endEffectorSubsystem.isCoralRollerAtSetpoint() &&
  // endEffectorSubsystem.isPivotAtSetpoint()
  //             && endEffectorSubsystem.isWristAtSetpoint() && endEffectorSubsystem.isCoralIntaked;
  //     // return false;
  // }

  @Override
  public void end(boolean interruped) {}
}
