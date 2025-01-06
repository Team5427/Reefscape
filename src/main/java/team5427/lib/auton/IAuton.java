package team5427.lib.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public interface IAuton {
    public static SequentialCommandGroup intake = new SequentialCommandGroup(
    // Add commands and Paths here
    );
    public static SequentialCommandGroup score = new SequentialCommandGroup(
    // Add commands and Paths here
    );
    // Add more SequentialCommandGroups here

    public static SequentialCommandGroup makeNewPath(String name, int nextPointer) {
        return new SequentialCommandGroup(
        // Add commands here
        // Base Paths off of name and nextPointer
        );
    }
}