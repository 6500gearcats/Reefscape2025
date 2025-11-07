  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot.commands;

  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj2.command.Command;
  import frc.robot.subsystems.DriveSubsystem;
  import frc.robot.subsystems.Vision;

  public class followTshirt extends Command {

    private Vision m_vision;
    private DriveSubsystem m_drive;
    private double targetY = 0;
    private double targetX = 2.0;

    public followTshirt(Vision vision, DriveSubsystem drive) {
      m_vision = vision;
      m_drive = drive;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(!m_vision.hasTarget()) {
        m_drive.drive(0,0,0,false);
        return;
      }
      double yaw = m_vision.getChosenYaw(18);
      double xDist = m_vision.getChosenRange(18);
      double currY = Math.tan(Math.toRadians(yaw)) * xDist;

      SmartDashboard.putNumber("Curr X", currY);
      SmartDashboard.putNumber("Dist away", xDist);

      System.out.println(currY);


      double xSpeed = (xDist - targetX) * -.167;
      double ySpeed = (currY - targetY) * 0.5;
      
      if(currY != 0 && yaw != 0) {
        m_drive.drive(xSpeed, ySpeed, (yaw*-1)/180, false); // X FOR SOME REASON IS FORWARD AND NOT SIDEWAYS 
      } 
      else {
        m_drive.drive(0,0,0,false);
      }
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
