// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithNearestAprilTag extends Command {
  /** Creates a new AlignWithAprilTag. */
  private Vision m_camera;
  private DriveSubsystem m_drive;

  private PIDController skewController;
  private PIDController yawController;

  public AlignWithNearestAprilTag( Vision camera, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_camera = camera;
    m_drive = drive;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double skew = m_camera.getBestZAxisRotation();
    double rotation = m_camera.getBestYaw();

    m_drive.drive(0, -rotation * 0.05, -skew * 0.05, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_camera.getBestZAxisRotation()) >= 3.05 && Math.abs(m_camera.getBestYaw()) <= 1) || m_camera.getBestZAxisRotation()==0;
  }
}
