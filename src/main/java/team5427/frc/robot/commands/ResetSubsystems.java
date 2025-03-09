package team5427.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.ClimbConstants;
import team5427.frc.robot.Constants.ProngEffectorConstants;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.Climb.ClimberSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

public class ResetSubsystems extends Command {

  private CascadeSubsystem cascadeSubsystem;
  private ClimberSubsystem climberSubsystem;
  private ProngSubsystem prongSubsystem;

  public ResetSubsystems() {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    prongSubsystem = ProngSubsystem.getInstance();
    climberSubsystem = ClimberSubsystem.getInstance();
    addRequirements(cascadeSubsystem);
  }

  @Override
  public void initialize() {
    cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kStowDistance);
    cascadeSubsystem.setPivotSetpoint(CascadeConstants.kStowRotation);
    prongSubsystem.setWristSetpoint(ProngEffectorConstants.kStowPosition);
    climberSubsystem.setSetpoint(ClimbConstants.kStowPosition);
    Climb.step = Climb.kReset;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
