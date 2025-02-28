package team5427.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.Constants.CascadeConstants;;

public class Climb extends Command {

    private static int step = 0;

    private CascadeSubsystem cascadeSubsystem;

    private static final int kPrep = 1;
    private static final int kActivate = 2;
    private static final int kClimb = 3;

    public Climb() {
        cascadeSubsystem = CascadeSubsystem.getInstance();
        addRequirements(cascadeSubsystem);
        step++;
    }

    @Override
    public void initialize() {
        if (step == kClimb) {
            cascadeSubsystem.setPivotSetpoint(CascadeConstants.kTempClimbRotation);
        }
    }
    
}
