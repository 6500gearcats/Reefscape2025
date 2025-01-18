// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Vision;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  XboxController m_driver = new XboxController(0);
  XboxController m_gunner = new XboxController(1);


  PhotonCamera temp_camera = new PhotonCamera("temp_camera");

  Vision vision = new Vision(temp_camera);
  //Temporarily adding this to
  DriveSubsystem m_robotDrive = new DriveSubsystem(vision);

  public RobotContainer() {
    NamedCommands.registerCommand("Place-Holder Coral Place", null);
    NamedCommands.registerCommand("Place-Holder Coral Grab", null);
    NamedCommands.registerCommand("Place-Holder Algae Grab", null);
    NamedCommands.registerCommand("Place-Holder Algae Processor", null);
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


    configureBindings();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driver.getLeftY(), 0.1), //0.1
                MathUtil.applyDeadband(-m_driver.getLeftX(), 0.1), //0.1
                MathUtil.applyDeadband(-m_driver.getRightX(), 0.1),
                !m_driver.getRightBumper()),
            m_robotDrive));
  }

  private void configureBindings() {

    new POVButton(m_driver, 90)
    .onTrue(
      new RunCommand(() -> m_robotDrive.drive(0,-.25,0,false)).withTimeout(0.75));

new POVButton(m_driver, 270)
    .onTrue(
      new RunCommand(() -> m_robotDrive.drive(0.0,.25,0,false)).withTimeout(0.75));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
