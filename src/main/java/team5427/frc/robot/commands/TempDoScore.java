package team5427.frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants.ProngEffectorConstants;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

public class TempDoScore extends Command {

  private CascadeSubsystem cascadeSubsystem;
  private ProngSubsystem prongSubsystem;

  private Distance dist;
  private Rotation2d rot;

  public TempDoScore(Distance dist, Rotation2d rot) {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    prongSubsystem = ProngSubsystem.getInstance();
    addRequirements(cascadeSubsystem, prongSubsystem);
    this.dist = dist;
    this.rot = rot;
  }

  @Override
  public void initialize() {
    cascadeSubsystem.setCascadeSetpoint(dist);
    cascadeSubsystem.setPivotSetpoint(Rotation2d.kZero);
    prongSubsystem.setWristSetpoint(rot);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    prongSubsystem.setWristSetpoint(ProngEffectorConstants.kStowPosition);
  }
}
