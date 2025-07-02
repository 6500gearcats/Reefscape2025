// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
public class RobotContainer {

  AprilTagFieldLayout field;
  Pose2d newPose = new Pose2d();

  //private final SendableChooser<Command> autoChooser;

  XboxController m_driver = new XboxController(0);
  XboxController m_gunner = new XboxController(1);

  
 
  Intake m_intake = new Intake();
  Shooter m_shooter = new Shooter();

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  //Temporarily adding this to
    /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driver.getLeftY() * -1,
                                                                () -> m_driver.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driver::getRightX)
                                                            .deadband(0.1)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

 /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driver::getRightX,
  m_driver::getRightY)
                                                           .headingWhile(true);

  public RobotContainer() {
   
    // Build an auto chooser. This will use Commands.none() as the default option.
    //autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    // *default command
    // 
    
  }

  private void configureBindings() {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // Configure your button bindings here
    //new JoystickButton(m_gunner, XboxController.Button.kA.value).whileTrue(new IntakeNote(m_intake).andThen(new IntakeCommand(m_intake, -0.2).withTimeout(0.2)));
    //new JoystickButton(m_gunner, XboxController.Button.kX.value).whileTrue(new Shoot(m_shooter));
    //new JoystickButton(m_gunner, XboxController.Button.kY.value).whileTrue(new Shoot(m_shooter).withTimeout(1.2).andThen((new ShootNote(m_intake, m_shooter))));
  }

  public Command getAutonomousCommand() {
    return null;
    //return autoChooser.getSelected();
  }

  public void resetRobotGyroAndOrientation() {
    //* Zero heading
  }
  
}
