// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SourceIntake.ShootCoral;
import frc.robot.subsystems.CoralHolder;
//import frc.robot.subsystems.GroundIntake;

public class RobotContainer {
  private final XboxController m_driver = new XboxController(0);
  private final XboxController m_gunner = new XboxController(1);

  //private final GroundIntake m_groundIntake = new GroundIntake();
  //private final Aligner m_aligner = new Aligner();
  private final CoralHolder m_coralHolder = new CoralHolder();
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_driver, Button.kA.value).whileTrue(new ShootCoral(m_coralHolder, 0.8)); // Outake coral
    new JoystickButton(m_driver, Button.kB.value).whileTrue(new ShootCoral(m_coralHolder, -0.8)); // Intake coral
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
