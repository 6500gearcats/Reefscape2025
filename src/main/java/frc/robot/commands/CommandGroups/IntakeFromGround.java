// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GroundIntake.FlipGroundIntakeOut;
import frc.robot.commands.GroundIntake.GrabCoral;
import frc.robot.subsystems.GroundIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromGround extends SequentialCommandGroup {
  /** Creates a new IntakeFromGround. */
  public IntakeFromGround(GroundIntake m_groundIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FlipGroundIntakeOut(m_groundIntake, -0.2),
      new GrabCoral(m_groundIntake, 0.5));
  }
}
