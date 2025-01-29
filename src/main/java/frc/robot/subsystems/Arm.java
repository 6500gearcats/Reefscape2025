// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  SparkMax m_armMotor = new SparkMax(0, MotorType.kBrushless);
  SparkMaxConfig m_config = new SparkMaxConfig();
  SparkAbsoluteEncoder m_armMotorEncoder = m_armMotor.getAbsoluteEncoder();
  double armPosition;
  //private ArmFeedforward armFeedforward = new ArmFeedforward(ArmConstants.kArm_kS, ArmConstants.kArm_kG, ArmConstants.kArm_kV);
  private PIDController armPIDcontroller1;

  /** Creates a new Arm. */
  public Arm() {
    armPIDcontroller1 = new PIDController(1, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //It's progamming's fault (:
    armPosition = getArmPosition();
    SmartDashboard.putNumber("Arm Position", armPosition);
  }

  public void spinArm(double speed) {
    m_armMotor.set(speed);
  }

  // Arm is at the top in coral collecter side
  public boolean atCoralMax() {
    return getArmPosition() > ArmConstants.kEncoderUpperThreshold;
  }

  // Arm is at the top in algae collecter side
  public boolean atAlgaeMax() {
    return getArmPosition() < ArmConstants.kEncoderLowerThreshold;
  }

  public void stop() {
    m_armMotor.set(0);
  }

  public double getArmPosition() {
    return m_armMotorEncoder.getPosition();
  }
    
  public void moveTo(double target) {
    spinArm(armPIDcontroller1.calculate(getArmPosition(), target)*ArmConstants.kArmForwardSpeed*10);
  }

    // This code might work - unsure because it uses a PID Controller that is from Revlib and we never utilized it last year (It was also initialized wrong last year)

   /**armPIDcontroller1.setReference(
                    target.getRadians(),
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot0,
                    armFeedforward.calculate(target.getRadians(), 0));
    }
                    */
}
