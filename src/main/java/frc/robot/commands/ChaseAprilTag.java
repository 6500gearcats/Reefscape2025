// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChaseAprilTag extends Command {
  /** Creates a new ChaseAprilTag. */
  Vision m_vision;
  DriveSubsystem m_robotDrive;
  PIDController pidController;
  public ChaseAprilTag(Vision m_vision, DriveSubsystem m_robotDrive) {
    this.m_vision = m_vision;
    this.m_robotDrive = m_robotDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = m_vision.getTargetDistance();
    System.out.println("Speed: " + distance * 0.005);
    double rotation = m_vision.getBestYaw();
    System.out.println("Rotation: " + -rotation * 0.005);
    m_robotDrive.drive(distance * 0.005, 0, -rotation * 0.05, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
