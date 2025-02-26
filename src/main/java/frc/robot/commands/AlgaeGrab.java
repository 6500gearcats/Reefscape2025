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
public class AlgaeGrab extends SequentialCommandGroup {
  /** Creates a new L4Sequence. */
  public AlgaeGrab(Arm m_arm, AlgaeIntake m_AlgaeIntake, Elevator m_elevator, DriveSubsystem m_drive) {
    addCommands(
      // Sets elevator and arm to L4 Coral position
      new SetArmAndElevatorPositions(m_elevator, m_arm, 0.36, 0.448),
      new InstantCommand(()->m_drive.drive(1, 0, 0, false)).withTimeout(.5),
      new IntakeAlgae(m_AlgaeIntake, -0.6)
    );
  }
}
