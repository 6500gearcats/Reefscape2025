// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class followTshirt extends Command {
  /** Creates a new followTshirt. */
  private Vision m_vision;
  private DriveSubsystem m_drive;
  private double targetX = 0;
  private double targetY = 2.0;

  public followTshirt(Vision vision, DriveSubsystem drive) {
    m_vision = vision;
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = m_vision.getBestYaw();
    double robotDist = m_vision.getBestRange();

    double xSpeed = (targetY - robotDist) * -.167;
    double ySpeed = (targetX - yaw) * -.167;

    m_drive.drive(xSpeed, ySpeed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
