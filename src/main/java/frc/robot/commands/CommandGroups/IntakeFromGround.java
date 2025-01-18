// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.GroundIntake.FlipGroundIntake;
import frc.robot.commands.GroundIntake.GrabCoral;
import frc.robot.subsytems.GroundIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromGround extends ParallelCommandGroup {
  /** Creates a new IntakeFromGround. */
  public IntakeFromGround(GroundIntake m_groundIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FlipGroundIntake(m_groundIntake),
      new GrabCoral(m_groundIntake, 0.1));
  }
}
