package team5427.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.ClimbConstants;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.Climb.ClimberSubsystem;

public class ResetSubsystems extends Command {

    private CascadeSubsystem cascadeSubsystem;
    private ClimberSubsystem climberSubsystem;

    public ResetSubsystems() {
        cascadeSubsystem = CascadeSubsystem.getInstance();
        climberSubsystem = ClimberSubsystem.getInstance();
        addRequirements(cascadeSubsystem, climberSubsystem);
    }

    @Override
    public void initialize() {
        cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kStowDistance);
        cascadeSubsystem.setPivotSetpoint(CascadeConstants.kStowRotation);
        climberSubsystem.setSetpoint(ClimbConstants.kStowPosition);
        Climb.step = Climb.kReset;
    }

    @Override
    public boolean isFinished() {
         return true;
    }
    
}
