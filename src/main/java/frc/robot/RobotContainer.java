// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignWithAprilTag;
import frc.robot.commands.SetAprilTagHorizontalOffset;
import frc.robot.commands.SetAprilTagVerticalOffset;
import frc.robot.commands.followTshirt;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  XboxController m_driver = new XboxController(0);
  XboxController m_gunner = new XboxController(1);

  PS4Controller m_pranav = new PS4Controller(0);
  

  GCPhotonVision m_PhotonCamera = new GCPhotonVision(new PhotonCamera("ArducamTwo"));
  GCLimelight m_Limelight = new GCLimelight("limelight-gca");
  Vision m_SimVision = new Vision(m_PhotonCamera);


  //PhotonCamera temp_camera = new PhotonCamera("ArducamTwo");
  //GCPhotonVision vision = new GCPhotonVision(temp_camera);
  Vision m_vision = new Vision(m_Limelight);
  
  //Temporarily adding this to
  DriveSubsystem m_robotDrive = new DriveSubsystem(m_PhotonCamera, m_vision);
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
                MathUtil.applyDeadband(-m_pranav.getLeftY(), 0.1), //0.1
                MathUtil.applyDeadband(-m_pranav.getLeftX(), 0.1), //0.1
                MathUtil.applyDeadband(-m_pranav.getRightX(), 0.1),
                !m_pranav.getR1Button()),
            m_robotDrive));
  }

  private void configureBindings() {
    // Configure your button bindings here
    //new JoystickButton(m_driver, XboxController.Button.kX.value).whileTrue(new FlipGroundIntake(m_groundIntake)).onFalse(new FlipGroundIntake(m_groundIntake));
    new JoystickButton(m_driver, Button.kA.value).onTrue(new AlignWithAprilTag(17, m_vision, m_robotDrive));
    new JoystickButton(m_driver, Button.kB.value).onTrue(new SetAprilTagHorizontalOffset(17, m_vision, m_robotDrive, .5));
    new JoystickButton(m_driver, Button.kY.value).onTrue(new SetAprilTagVerticalOffset(17, m_vision, m_robotDrive, 0));

    new JoystickButton(m_pranav, PS4Controller.Button.kCross.value).whileTrue(new followTshirt(m_SimVision, m_robotDrive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
