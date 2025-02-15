// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithSelectedAprilTag extends Command {
  /** Creates a new AlignWithAprilTag. */
  private Vision m_camera;
  private DriveSubsystem m_drive;

  public AlignWithSelectedAprilTag(Vision camera, DriveSubsystem drive) {
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
    double skew = m_camera.getZAxisRotation(Vision.chosenTag);
    double rotation = m_camera.getChosenYaw(Vision.chosenTag);

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
    return (Math.abs(m_camera.getZAxisRotation(Vision.chosenTag)) >= 1 && Math.abs(m_camera.getChosenYaw(Vision.chosenTag)) <= .2) || m_camera.getZAxisRotation(Vision.chosenTag)==0;
  }
}
