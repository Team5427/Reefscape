package team5427.frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.Constants.CascadeConstants;

public class MoveElevatorTo extends Command {

  private CascadeSubsystem cascadeSubsystem;
  private Distance distance;

  public MoveElevatorTo(Distance distance) {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    addRequirements(cascadeSubsystem);

    this.distance = distance;
  }

  @Override
  public void initialize() {
    cascadeSubsystem.setCascadeSetpoint(distance);
    cascadeSubsystem.setPivotSetpoint(CascadeConstants.kTempActiveRotation);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    // cascadeSubsystem.setPivotSetpoint(Rotation2d.fromDegrees(0));
    // cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kStowDistance);
  }
}
