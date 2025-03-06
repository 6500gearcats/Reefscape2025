// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetElevatorHeight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmAndElevatorPositions extends ParallelCommandGroup {
  /** Creates a new SetArmAndElevatorPositions. */
  public SetArmAndElevatorPositions(Elevator m_elevator, Arm m_arm, double m_height, double m_angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetElevatorHeight(m_elevator, m_height), new SetArmPosition(m_arm, m_angle));
  }

  public SetArmAndElevatorPositions(Elevator m_elevator, Arm m_arm, double m_height, double m_angle, double elevatorMinSpeed, double armMinSpeed, double elevatorMaxSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetElevatorHeight(m_elevator, m_height, elevatorMinSpeed, elevatorMaxSpeed), new SetArmPosition(m_arm, m_angle, armMinSpeed), new WaitCommand(0.5));

  }
}
