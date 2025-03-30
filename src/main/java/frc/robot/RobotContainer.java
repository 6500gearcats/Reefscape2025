// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralHolder;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.utility.GCLimelight;
import frc.robot.utility.GCPhotonVision;
import frc.robot.utility.LimelightHelpers;
import frc.robot.commands.AlgaeGrab;
import frc.robot.commands.AlgaeSequence;
import frc.robot.commands.CoralGrab;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.L4Sequence;
import frc.robot.commands.MoveCoral;
import frc.robot.commands.OutakeAlgae;
import frc.robot.commands.ProportionalAlign;
import frc.robot.commands.ProportionalAlignCoralStation;
import frc.robot.commands.ProportionalAlignTeleop;
import frc.robot.commands.SetArmSpeed;
import frc.robot.commands.SetClimberSpeed;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.commands.SnailEnable;
import frc.robot.commands.TurboEnable;
import frc.robot.commands.SetArmAndElevatorPositions;
import frc.robot.commands.SetArmAndElevatorPositionsSource;

public class RobotContainer {

  // AprilTagFieldLayout field;
  // Pose2d newPose = new Pose2d();

  XboxController m_driver = new XboxController(0);
  XboxController m_gunner = new XboxController(1);

  GCPhotonVision m_PhotonCamera = new GCPhotonVision(new PhotonCamera("ArducamTwo"));
  GCLimelight m_Limelight = new GCLimelight("limelight-gcc");

  AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
  Arm m_arm = new Arm();
  Climber m_climber = new Climber();
  CoralHolder m_CoralHolder = new CoralHolder();

  boolean fieldRelative = true;

  PhotonCamera temp_camera = new PhotonCamera("ArducamTwo");
  GCPhotonVision vision = new GCPhotonVision(temp_camera);
  //Vision m_vision = new Vision(m_Limelight);

  DriveSubsystem m_robotDrive = new DriveSubsystem(m_PhotonCamera, m_Limelight);

  Elevator m_elevator = new Elevator();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("CoralPlace", new L4Sequence(m_arm, m_CoralHolder, m_elevator, m_robotDrive));
    NamedCommands.registerCommand("CoralGrab", new CoralGrab(m_arm, m_CoralHolder, m_elevator, m_robotDrive));
    NamedCommands.registerCommand("AlgaeGrab", new AlgaeGrab(m_arm, m_AlgaeIntake, m_elevator, m_robotDrive));
    NamedCommands.registerCommand("RaiseElevatorL4", new SetArmAndElevatorPositions(m_elevator, m_arm, 0.734, .555, 0.6, 0.4, -5).andThen(new SetElevatorSpeed(m_elevator, () -> -0.6).withTimeout(0.1)));
    NamedCommands.registerCommand("LowerElevatorL4", new SetArmAndElevatorPositions(m_elevator, m_arm, 0.18, 0.1, 0.6, 0.4, -4));
    NamedCommands.registerCommand("LowerElevatorAlgae", new SetArmAndElevatorPositions(m_elevator, m_arm, 0.026, 0.361, 0.3, 0.4, -2));
    NamedCommands.registerCommand("AlgaeProcessor", new AlgaeSequence(m_arm, m_AlgaeIntake, m_elevator, m_robotDrive));
    NamedCommands.registerCommand("SetPreElevator", new SetArmAndElevatorPositions(m_elevator, m_arm, 0.18, 0.36, 0.6, 0.4, -4));
    NamedCommands.registerCommand("ProportionalAlignLeft", new ProportionalAlign(m_robotDrive, -0.15, .475, 2));
    NamedCommands.registerCommand("ProportionalAlignRight", new ProportionalAlign(m_robotDrive, 0.15, .475, 2));
    NamedCommands.registerCommand("ProportionalAlignAlgae", new ProportionalAlign(m_robotDrive, 0, .475, 2));
    NamedCommands.registerCommand("MiddleTaxi", new RunCommand(() -> m_robotDrive.drive(-1, 0, 0, false), m_robotDrive).withTimeout(1).andThen(new RunCommand(() -> m_robotDrive.drive(0,0,0,false))));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser("3CoralTop");
    SmartDashboard.putData("Autonomous Chooser", autoChooser);

    m_robotDrive.zeroHeading();
    LimelightHelpers.SetRobotOrientation(
        "limelight-gcc", m_robotDrive.getAngle(), 0, 0, 0, 0, 0);
    LimelightHelpers.setCameraPose_RobotSpace("limelight-gcc", -0.318, 0.177, 0.29, 0, 6, 180);

    configureBindings();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driver.getLeftY(), 0.1) * 0.8, // 0.1
                MathUtil.applyDeadband(-m_driver.getLeftX(), 0.1) * 0.8, // 0.1
                MathUtil.applyDeadband(-m_driver.getRightX(), 0.1) * 0.8,
                !m_driver.getRightBumper(), "Drive Train - Controller"),
            m_robotDrive));
  }

  private void configureBindings() {

    //new JoystickButton(m_driver, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));

    new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value).whileTrue(new TurboEnable(m_robotDrive));
    new JoystickButton(m_driver, XboxController.Button.kA.value).whileTrue(new SnailEnable(m_robotDrive));
    // TODO: UPDATE BUTTONS BASED ON REQUESTED BUTTONS
    // new JoystickButton(m_driver, XboxController.Button.kX.value).whileTrue(new
    // FlipGroundIntake(m_groundIntake)).onFalse(new
    // FlipGroundIntake(m_groundIntake));
    new Trigger(() -> m_gunner.getRightY() > 0.5)
        .whileTrue(new SetElevatorSpeed(m_elevator, () -> m_gunner.getRightY() * 0.85));
    new Trigger(() -> m_gunner.getRightY() < -0.5)
        .whileTrue(new SetElevatorSpeed(m_elevator, () -> m_gunner.getRightY() * 0.85));
    new Trigger(() -> m_gunner.getLeftTriggerAxis() > 0.3).whileTrue(new IntakeAlgae(m_AlgaeIntake, -0.7));
    new Trigger(() -> m_gunner.getRightTriggerAxis() > 0.3).whileTrue(new OutakeAlgae(m_AlgaeIntake, 1.1));

    new Trigger(() -> m_gunner.getLeftY() > 0.2).whileTrue(new SetArmSpeed(m_arm, () -> m_gunner.getLeftY() * -0.5));
    new Trigger(() -> m_gunner.getLeftY() < -0.2).whileTrue(new SetArmSpeed(m_arm, () -> m_gunner.getLeftY() * -0.5));

    new JoystickButton(m_gunner, XboxController.Button.kStart.value).whileTrue(
        new SetArmAndElevatorPositions(m_elevator, m_arm, 0, 0.168).andThen(new SetClimberSpeed(m_climber, 0.9)));
    new JoystickButton(m_gunner, XboxController.Button.kBack.value).whileTrue(new SetClimberSpeed(m_climber, -0.6));

    new JoystickButton(m_gunner, XboxController.Button.kLeftBumper.value)
        .whileTrue(new MoveCoral(m_CoralHolder, 0.5, true));
    new JoystickButton(m_gunner, XboxController.Button.kRightBumper.value)
        .whileTrue(new MoveCoral(m_CoralHolder, -0.8, false).withTimeout(0.2)
            .andThen(new SetArmSpeed(m_arm, () -> 0.6).withTimeout(0.6)));

    // Coral L3
    new JoystickButton(m_gunner, XboxController.Button.kX.value)
        .whileTrue(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.36, 0.547));
    // Coral L4
    new JoystickButton(m_gunner, XboxController.Button.kY.value)
        .whileTrue(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.738, .555).andThen(new SetElevatorSpeed(m_elevator, () -> -0.3).withTimeout(0.1)));//.withTimeout(.1));

    // Coral L2
    new JoystickButton(m_gunner, XboxController.Button.kB.value)
        .whileTrue(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.3, 0.1)
            .andThen(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.19, 0.56, 0.2, 0.4, -2)));

    // Source
    new JoystickButton(m_gunner, XboxController.Button.kA.value)
        .whileTrue(new SetArmAndElevatorPositionsSource(m_elevator, m_arm, 0.18, 0.1, 0.4, 0.4, -2));

    // Algae L3
    new POVButton(m_gunner, 270).whileTrue(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.43, 0.381));

    // Algae L2
    new POVButton(m_gunner, 90).whileTrue(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.36, 0.399));

    // Net
    new POVButton(m_gunner, 0).whileTrue(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.738, 0.195));

    // Processor
    new POVButton(m_gunner, 180).whileTrue(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.026, 0.361, 0.3, 0.4, -2));
    new JoystickButton(m_driver, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> resetRobotGyroAndOrientation()));
    new POVButton(m_driver, 180).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    // previous yOffset = 0.75
    //new Trigger((() -> m_driver.getLeftTriggerAxis() > 0.2)).whileTrue(new ProportionalAlign(m_robotDrive, -0.15, .485, 2));
    //new Trigger((() ->  m_driver.getRightTriggerAxis() > 0.2)).whileTrue(new ProportionalAlign(m_robotDrive, 0.2, .485, 2));
    new Trigger((() -> m_driver.getLeftTriggerAxis() > 0.2)).whileTrue(new ProportionalAlignTeleop(m_robotDrive, -0.15, .650, 2).andThen(new ProportionalAlignTeleop(m_robotDrive, -0.15, .500, 2)));
    new Trigger((() ->  m_driver.getRightTriggerAxis() > 0.2)).whileTrue(new ProportionalAlignTeleop(m_robotDrive, 0.2, .650, 2).andThen(new ProportionalAlignTeleop(m_robotDrive, 0.2, .500, 2)));
    new POVButton(m_driver, 0).whileTrue(new ProportionalAlign(m_robotDrive, 0, .475));

    // Auto score L4 left
    //new JoystickButton(m_driver, XboxController.Button.kX.value).whileTrue((new ProportionalAlign(m_robotDrive, -0.15, 0.535)));//.andThen(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.735, .555)).andThen(new ProportionalAlign(m_robotDrive, -0.15, 0.45)).andThen(new MoveCoral(m_CoralHolder, -0.5, false)).withTimeout(0.2).andThen(new SetArmSpeed(m_arm, () -> 0.4)).withTimeout(0.6));

    // Auto score L4 right
    //new JoystickButton(m_driver, XboxController.Button.kB.value).whileTrue((new ProportionalAlign(m_robotDrive, 0.2, 0.535)));//.andThen(new SetArmAndElevatorPositions(m_elevator, m_arm, 0.735, .555)).andThen(new ProportionalAlign(m_robotDrive, 0.2, 0.45)).andThen(new MoveCoral(m_CoralHolder, -0.5, false)).withTimeout(0.2).andThen(new SetArmSpeed(m_arm, () -> 0.4)).withTimeout(0.6));
    
    // Driver right coral station
    new POVButton(m_driver, 90).whileTrue(new ProportionalAlignCoralStation(m_robotDrive, 0.4, .800, 3).andThen(new ProportionalAlignCoralStation(m_robotDrive, 0.4, .480, 2)));

    // Driver middle coral station
    new POVButton(m_driver, 0).whileTrue(new ProportionalAlignCoralStation(m_robotDrive, 0, .800, 3).andThen(new ProportionalAlignCoralStation(m_robotDrive, 0, .480, 2)));

    // Driver left coral station
    new POVButton(m_driver, 270).whileTrue(new ProportionalAlignCoralStation(m_robotDrive, -0.4, .800, 3).andThen(new ProportionalAlignCoralStation(m_robotDrive, -0.4, .480, 2)));

  }

  public Command getAutonomousCommand() {
    //Optional<Alliance> alliance = DriverStation.getAlliance();
    //if (alliance.isPresent()) {
      //if (alliance.get().equals(Alliance.Blue)) {
       return autoChooser.getSelected();
      /*} else {
        return new RunCommand(() -> m_robotDrive.drive(-1, 0, 0, false), m_robotDrive).withTimeout(1).andThen(new RunCommand(() -> m_robotDrive.drive(0,0,0,false)));
      }*/
    //}
    //return new WaitCommand(0.5);
  }

  public void resetRobotGyroAndOrientation() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get().equals(Alliance.Blue)) {
        m_robotDrive.zeroHeading();
        LimelightHelpers.SetRobotOrientation("limelight-gcc", m_robotDrive.getAngle(), 0, 0, 0, 0, 0);
        LimelightHelpers.setCameraPose_RobotSpace("limelight-gcc", -0.318, 0.177, 0.29, 0, 6, 180);
      } else {
        m_robotDrive.zeroHeading();
        LimelightHelpers.SetRobotOrientation("limelight-gcc", m_robotDrive.getAngle() + 180, 0, 0, 0, 0, 0);
        LimelightHelpers.setCameraPose_RobotSpace("limelight-gcc", -0.318, 0.177, 0.29, 0, 6, 180);
      }
    }
  }
}
