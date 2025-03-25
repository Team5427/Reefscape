package team5427.frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.RawIntakeConfiguration;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.SupremeEffector.SupremeSubsystem;

public class JITB_Intake extends Command {

  private CascadeSubsystem cascadeSubsystem;
  private SupremeSubsystem jitbIntake;
  private RawIntakeConfiguration configuration;

  public JITB_Intake(RawIntakeConfiguration configuration) {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    jitbIntake = SupremeSubsystem.getInstance();
    addRequirements(jitbIntake);
    this.configuration = configuration;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    cascadeSubsystem.setCascadeSetpoint(configuration.getCascadeHeight());
    cascadeSubsystem.setPivotSetpoint(configuration.getCascadeAngle());
    jitbIntake.setAlgaeRollerSetpoint(configuration.getAlgaeRollerSpeeds());
    jitbIntake.setCoralRollerSetpoint(configuration.getCoralRollerSpeeds());
    jitbIntake.setPivotSetpoint(configuration.getWristAngle());
  }

  @Override
  public boolean isFinished() {
    return jitbIntake.isAlgaeIntaked() || jitbIntake.isCoralIntaked();
  }
}
