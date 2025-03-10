package team5427.frc.robot.io;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team5427.frc.robot.Constants.BlinkinConstants;
import team5427.frc.robot.SuperStructureEnum.CascadeStates;
import team5427.frc.robot.subsystems.LightsSubsystem;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;

public class LightTriggers {
  private LightsSubsystem lights = LightsSubsystem.getInstance();

  public LightTriggers() {
    Trigger cascadeClimb = new Trigger(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
          return CascadeSubsystem.state == CascadeStates.CLIMBING;
        }
    });

    Trigger shot = new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return CascadeSubsystem.state == CascadeStates.SHOOTING;
      }
    });

    Trigger score = new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return CascadeSubsystem.state == CascadeStates.SCORING;
      }
    });

    Trigger idle = new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return CascadeSubsystem.state != CascadeStates.SCORING
          && CascadeSubsystem.state != CascadeStates.SHOOTING
          && CascadeSubsystem.state != CascadeStates.CLIMBING;
      }
    });

    cascadeClimb.whileTrue(
      new InstantCommand(() -> {
        lights.setPattern(BlinkinConstants.kCp1Strobe);
      })
    );

    shot.whileTrue(
      new InstantCommand(() -> {
        lights.setPattern(BlinkinConstants.kCp2LarsonScanner);
      })
    );

    score.whileTrue(
      new InstantCommand(() -> {
        lights.setPattern(BlinkinConstants.kGreen);
      })
    );
        
    idle.whileTrue(
      new InstantCommand(() -> {
        lights.setPattern(BlinkinConstants.kBreath);
      })
    );
  }
}