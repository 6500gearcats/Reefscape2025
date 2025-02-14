// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GCPoseEstimator;
import frc.robot.subsystems.Vision;
import frc.robot.GCPhotonVision;
import frc.robot.commands.AlignWithAprilTag;

import frc.robot.commands.SetAprilTagHorizontalOffset;
import frc.robot.commands.SetAprilTagVerticalOffset;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
@SuppressWarnings("unused")
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  XboxController m_driver = new XboxController(0);
  XboxController m_gunner = new XboxController(1);

  // * test controllers
  PS4Controller  m_driverP = new PS4Controller (0);
  PS4Controller  m_gunnerP = new PS4Controller(1);

  GCPhotonVision m_PhotonCamera = new GCPhotonVision(new PhotonCamera("ArducamTwo"));
  GCLimelight m_LimelightA = new GCLimelight("limelight-gcc");
  GCLimelight m_LimelightB= new GCLimelight("limelight-gca");


  PhotonCamera temp_camera = new PhotonCamera("ArducamTwo");
  GCPhotonVision photonVision = new GCPhotonVision(temp_camera);

  Vision m_visionLL1 = new Vision(m_LimelightA, m_LimelightB);
  Vision m_visionLL2 = new Vision(m_LimelightA, m_LimelightB);

  //Temporarily adding this to
  DriveSubsystem m_robotDrive = new DriveSubsystem(m_PhotonCamera, m_visionLL1);

  public RobotContainer() {

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


    configureBindings();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverP.getLeftY(), 0.1), //0.1
                MathUtil.applyDeadband(-m_driverP.getLeftX(), 0.1), //0.1
                MathUtil.applyDeadband(-m_driverP.getRightX(), 0.1),
                !m_driver.getRightBumper()),
            m_robotDrive));
  }

  private void configureBindings() {
    // Configure your button bindings here

    //TODO: UPDATE BUTTONS BASED ON REQUESTED BUTTONS
    // * Only sending in one limelight to the folloeing april tag commands
    //new JoystickButton(m_driver, XboxController.Button.kX.value).whileTrue(new FlipGroundIntake(m_groundIntake)).onFalse(new FlipGroundIntake(m_groundIntake));
    
    new JoystickButton(m_driverP, Button.kB.value).whileTrue(new SetAprilTagHorizontalOffset(14, m_visionLL1, m_robotDrive, .5));
    new JoystickButton(m_driverP, Button.kY.value).whileTrue(new SetAprilTagVerticalOffset(14, m_visionLL1, m_robotDrive, 0));
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
