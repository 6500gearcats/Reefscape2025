// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandGroups.IntakeFromGround;
import frc.robot.commands.GroundIntake.FlipGroundIntake;
import frc.robot.subsytems.GroundIntake;

public class RobotContainer {
  @SuppressWarnings("unused")
  private final XboxController m_driver = new XboxController(0);
  private final XboxController m_gunner = new XboxController(1);
  private final GroundIntake m_groundIntake = new GroundIntake();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_gunner, Button.kA.value).whileTrue(new IntakeFromGround(m_groundIntake));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
