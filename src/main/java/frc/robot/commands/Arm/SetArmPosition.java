// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetArmPosition extends Command {
  /** Creates a new SetArmPOsition. */
  private Arm m_arm;
  private double m_armAngle;

  public SetArmPosition(Arm arm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_armAngle = angle;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // Check to make sure this first condition isn't just useless
      if((m_armAngle > ArmConstants.KEncoderDeadbandThreshold)
      && (m_armAngle < ArmConstants.kEncoderUpperThreshold) && (m_armAngle > ArmConstants.kEncoderLowerThreshold))
      {
        if((m_arm.getArmPosition() > m_armAngle || m_arm.getArmPosition() < m_armAngle) && getError() > 0.001) {
          m_arm.moveTo(m_armAngle);
        }
        SmartDashboard.putString("Runningarm:", "MovingToAngle " + m_armAngle);
      }
      else
      {
        m_arm.stop();
      }
    }
  
         
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_arm.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return Math.abs(m_arm.getArmPosition() - m_armAngle) < 0.001;
    }
  
    private double getError() {
      double error = Math.abs(m_arm.getArmPosition() - m_armAngle) ;
      SmartDashboard.putNumber("arm Error", error);
      return error;
    }
}  