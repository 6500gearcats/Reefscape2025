// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Algae.IntakeAlgae;
import frc.robot.commands.Aligner.AlignCoral;
import frc.robot.commands.CommandGroups.AlignAndFeed;
import frc.robot.commands.CommandGroups.IntakeFromGround;
import frc.robot.commands.GroundIntake.FlipGroundIntake;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Aligner;
import frc.robot.subsystems.CoralHolder;
import frc.robot.subsystems.GroundIntake;

public class RobotContainer {

  private final XboxController m_driver = new XboxController(0);
  private final XboxController m_gunner = new XboxController(1);

  private final AlgaeIntake m_algaeIntake = new AlgaeIntake();

  private final GroundIntake m_groundIntake = new GroundIntake();
  private final Aligner m_aligner = new Aligner();
  private final CoralHolder m_coralHolder = new CoralHolder();
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_gunner, Button.kA.value).whileTrue(new IntakeFromGround(m_groundIntake))
      .onFalse(new AlignAndFeed(m_groundIntake, m_aligner, m_coralHolder));
    new JoystickButton(m_gunner, Button.kB.value).whileTrue(new IntakeAlgae(m_algaeIntake, .05));
    new JoystickButton(m_gunner, Button.kY.value).whileTrue(new AlignCoral(m_aligner, .5));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
