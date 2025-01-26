// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.CommandGroups.AlignAndFeed;
import frc.robot.Commands.CommandGroups.IntakeFromGround;
import frc.robot.Commands.GroundIntake.FlipGroundIntake;
import frc.robot.Subsystems.Aligner;
import frc.robot.Subsystems.CoralHolder;
import frc.robot.Subsystems.GroundIntake;
import frc.robot.Subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;

public class RobotContainer {

  private final XboxController m_driver = new XboxController(0);
  private final XboxController m_gunner = new XboxController(1);

  private final GroundIntake m_groundIntake = new GroundIntake();
  private final Aligner m_aligner = new Aligner();
  private final CoralHolder m_coralHolder = new CoralHolder();

  //TODO: Fix the Photon Camera, This is temp declaration
  private PhotonCamera m_camera = new PhotonCamera("photonvision");
  Vision m_vision = new Vision(m_camera);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision);
  
  public RobotContainer() {

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
    new JoystickButton(m_gunner, Button.kA.value).whileTrue(new IntakeFromGround(m_groundIntake))
      .onFalse(new AlignAndFeed(m_groundIntake, m_aligner, m_coralHolder));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
