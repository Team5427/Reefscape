package team5427.lib.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonManager implements IAuton {
  private static AutonManager instance;
  private SequentialCommandGroup currentCommandGroup;

  public AutonManager(SequentialCommandGroup commandGroup) {
    this.currentCommandGroup = commandGroup;
  }

  public AutonManager() {
    this.currentCommandGroup = null;
  }

  public static AutonManager getInstance() {
    if (instance == null) {
      instance = new AutonManager();
    }
    return instance;
  }
}
