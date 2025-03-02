package team5427.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.ClimbConstants;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.Climb.ClimberSubsystem;

public class Climb extends Command {

  public static int step = 0;

  private CascadeSubsystem cascadeSubsystem;
  private ClimberSubsystem climberSubsystem;

  public static final int kReset = 0;
  public static final int kPrep = 1;
  public static final int kActivate = 2;
  public static final int kClimb = 3;

  public Climb() {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    climberSubsystem = ClimberSubsystem.getInstance();
    addRequirements(cascadeSubsystem, climberSubsystem);
  }

  @Override
  public void initialize() {
    step += 1;
    Logger.recordOutput("Climber Step", step);
    switch (step) {
      case kPrep:
        cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kStowDistance);
        cascadeSubsystem.setPivotSetpoint(CascadeConstants.kClimbPrepRotation);
        climberSubsystem.setSetpoint(ClimbConstants.kPrepPosition);
        break;
      case kActivate:
        cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kStowDistance);
        cascadeSubsystem.setPivotSetpoint(CascadeConstants.kClimbPrepRotation);
        climberSubsystem.setSetpoint(ClimbConstants.kActivePosition);
        break;
      case kClimb:
        cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kStowDistance);
        cascadeSubsystem.setPivotSetpoint(CascadeConstants.kTempClimbRotation);
        climberSubsystem.setSetpoint(ClimbConstants.kActivePosition);
        break;
      default:
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
