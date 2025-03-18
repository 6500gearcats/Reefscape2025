// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralHolder;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4Sequence extends SequentialCommandGroup {
  /** Creates a new L4Sequence. */
  public L4Sequence(Arm m_arm, CoralHolder m_CoralHolder, Elevator m_elevator, DriveSubsystem m_drive) {
    addCommands(
      // Sets elevator and arm to L4 Coral position
      // new SetArmAndElevatorPositions(m_elevator, m_arm, 0.715, .561),
      // Runs forward because path allows room for elevator and arm to raise first
      //new InstantCommand(()->m_drive.drive(-1, 0, 0, false)).withTimeout(.5),
      // Outtakes coral (hopefully onto L4)
      // TODO: Add an isFinished into MoveCoral so that we don't have to use withTimeout
      new MoveCoral(m_CoralHolder, -0.5, false).withTimeout(0.2),
      new SetArmSpeed(m_arm, () -> .6).withTimeout(.4),
      new WaitCommand(0.25)
      // Runs backward to allow elevator room to go back down
      //new RunCommand(()->m_drive.drive(0.4, 0, 0, false)).withTimeout(.05),
      // Moves elevator and arm back down to source
    );
  }
}
