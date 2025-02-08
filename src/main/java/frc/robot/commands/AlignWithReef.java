// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignWithReef extends SequentialCommandGroup {
  /** Creates a new AlignWithReef. */
  public AlignWithReef(IntSupplier fiducialID, Vision camera, DriveSubsystem drive) {
    addCommands(
      new AlignWithAprilTag(fiducialID, camera, drive),
      new SetAprilTagHorizontalOffset(fiducialID.getAsInt(), camera, drive, .5),
      new SetAprilTagVerticalOffset(fiducialID.getAsInt(), camera, drive, 0)
    );
  }
}

