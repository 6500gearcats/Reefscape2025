// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Aligner.AlignCoral;
import frc.robot.commands.GroundIntake.FlipGroundIntake;
import frc.robot.commands.GroundIntake.GrabCoral;
import frc.robot.commands.SourceIntake.ShootCoral;
import frc.robot.subsystems.Aligner;
import frc.robot.subsystems.CoralHolder;
import frc.robot.subsystems.GroundIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndFeed extends ParallelDeadlineGroup {
  /** Creates a new AlignAndFeed. */
  public AlignAndFeed(GroundIntake m_groundIntake, Aligner m_aligner, CoralHolder m_coralHolder) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().

    super(new InstantCommand(() -> m_coralHolder.getFlipSwitchValue()));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FlipGroundIntake(m_groundIntake)
        .andThen(new GrabCoral(m_groundIntake, -0.1).withTimeout(0.2)), 
      new AlignCoral(m_aligner, 1),
      new ShootCoral(m_coralHolder, 0.01));
  }
}
