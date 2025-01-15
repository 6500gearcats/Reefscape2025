// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  XboxController m_driver = new XboxController(0);
  XboxController m_gunner = new XboxController(1);

  PhotonCamera temp_camera = new PhotonCamera("temp_camera");
  

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    // Configure your button bindings here

    //TODO: UPDATE BUTTONS BASED ON REQUESTED BUTTONS
    //new JoystickButton(m_driver, XboxController.Button.kX.value).whileTrue(new FlipGroundIntake(m_groundIntake)).onFalse(new FlipGroundIntake(m_groundIntake));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
