// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4Sequence extends SequentialCommandGroup {
  /** Creates a new L4Sequence. */
  public L4Sequence(Arm m_arm, AlgaeIntake m_algaeIntake, Elevator m_elevator, double speed) {
    addCommands(
      new SetArmAndElevatorPositions(m_elevator, m_arm, 0.65, 0.164),
      new OutakeAlgae(m_algaeIntake, speed)
    );
  }
}
