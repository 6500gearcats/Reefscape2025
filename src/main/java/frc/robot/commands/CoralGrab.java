// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  public CoralGrab(Arm m_arm, CoralHolder m_CoralHolder, Elevator m_elevator, DriveSubsystem m_drive) {
    addCommands(
      // This code assumes L4Sequence has just run and the elevator/arm are already in correct positions 
      // Intakes coral
      // TODO: REALLY need to add an isFinished into MoveCoral so that we don't have to use withTimeout
      new MoveCoral(m_CoralHolder, 0.5, true).withTimeout(2)
    );
  }
}
