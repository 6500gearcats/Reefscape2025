// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralHolder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.Vision;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.MoveCoral;
import frc.robot.commands.OutakeAlgae;
import frc.robot.commands.SetArmSpeed;
import frc.robot.commands.SetClimberSpeed;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.commands.TurboEnable;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  XboxController m_driver = new XboxController(0);
  XboxController m_gunner = new XboxController(1);

  AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
  Arm m_arm = new Arm();
  Climber m_climber = new Climber();
  CoralHolder m_CoralHolder = new CoralHolder();

  PhotonCamera temp_camera = new PhotonCamera("temp_camera");

  Vision vision = new Vision(temp_camera);
  //Temporarily adding this to
  DriveSubsystem m_robotDrive = new DriveSubsystem(vision);

  Elevator m_elevator = new Elevator();

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
                MathUtil.applyDeadband(-m_driver.getLeftY() *.8, 0.1), //0.1
                MathUtil.applyDeadband(-m_driver.getLeftX() *.8, 0.1), //0.1
                MathUtil.applyDeadband(-m_driver.getRightX() *.8, 0.1),
                !m_driver.getRightBumper()),
            m_robotDrive));
  }

  private void configureBindings() {
    // Configure your button bindings here

    new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value).whileTrue(new TurboEnable(m_robotDrive));
    //TODO: UPDATE BUTTONS BASED ON REQUESTED BUTTONS
    //new JoystickButton(m_driver, XboxController.Button.kX.value).whileTrue(new FlipGroundIntake(m_groundIntake)).onFalse(new FlipGroundIntake(m_groundIntake));
    new Trigger(() -> m_gunner.getRightY() > 0.5).whileTrue(new SetElevatorSpeed(m_elevator, 0.3));
    new Trigger(() -> m_gunner.getRightY() < -0.5).whileTrue(new SetElevatorSpeed(m_elevator, -0.4));
    new JoystickButton(m_driver, XboxController.Button.kA.value).whileTrue(new OutakeAlgae(m_AlgaeIntake, 0.6));
    new JoystickButton(m_driver, XboxController.Button.kB.value).whileTrue(new IntakeAlgae(m_AlgaeIntake, -0.6));

    new Trigger(() -> m_gunner.getLeftY() > 0.2).whileTrue(new SetArmSpeed(m_arm, -0.25));
    new Trigger(() -> m_gunner.getLeftY() < -0.2).whileTrue(new SetArmSpeed(m_arm, 0.25));

    new JoystickButton(m_driver, XboxController.Button.kY.value).whileTrue(new SetClimberSpeed(m_climber, 0.1));

    new JoystickButton(m_gunner, XboxController.Button.kA.value).whileTrue(new MoveCoral(m_CoralHolder, -0.5, true));
    new JoystickButton(m_gunner, XboxController.Button.kB.value).whileTrue(new MoveCoral(m_CoralHolder, 0.5, false));
    new JoystickButton(m_gunner, XboxController.Button.kX.value).whileTrue(new SetElevatorHeight(m_elevator, 0.15));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
