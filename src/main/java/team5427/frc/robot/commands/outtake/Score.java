package team5427.frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.RawScoringConfiguration;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

// import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem.EETask;

public class Score extends Command {

  private CascadeSubsystem cascadeSubsystem;
  private ProngSubsystem prongSubsystem;

  private RawScoringConfiguration config;

  public Score(RawScoringConfiguration config) {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    prongSubsystem = ProngSubsystem.getInstance();
    addRequirements(cascadeSubsystem, prongSubsystem);
    this.config = config;
  }

  @Override
  public void initialize() {

    // ProngSubsystem.task = EETask.EJECTING;
    // LimelightHelpers.setLEDMode_ForceOn(VisionConstants.kProngLimelightName);
    cascadeSubsystem.setPivotSetpoint(config.getCascadeAngle());
    cascadeSubsystem.setCascadeSetpoint(config.getCascadeHeight());
    prongSubsystem.setWristSetpoint(config.getWristAngle());
  }

  @Override
  public boolean isFinished() {
    // this commented code below is a bandaid, will be changing PID values in the future to not need
    // this
    // if(RobotState.isAutonomous()){
    //   return cascadeSubsystem.cascadeAtGoal(CascadeConstants.kCascadeToleranceAutonomous);
    // }
    return cascadeSubsystem.cascadeAtGoal();
  }

  @Override
  public void end(boolean interrupted) {
    // prongSubsystem.setWristSetpoint(ProngEffectorConstants.kStowPosition);
    System.out.println("At Goal");
    // LimelightHelpers.setLEDMode_ForceOff(VisionConstants.kProngLimelightName);
  }
}
