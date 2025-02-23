// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralHolder;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralGrab extends SequentialCommandGroup {
  /** Creates a new L4Sequence. */
  public CoralGrab(Arm m_arm, CoralHolder m_CoralHolder, Elevator m_elevator) {
    addCommands(
      // Sets elevator and arm to L4 Coral position
      new SetArmAndElevatorPositions(m_elevator, m_arm, 0.16, 0.1),
      new MoveCoral(m_CoralHolder, -0.5, true)
    );
  }
}
