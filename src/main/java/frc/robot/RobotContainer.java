// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.SetArmSpeed;
//import frc.robot.commands.CommandGroups.AlignAndFeed;
//import frc.robot.commands.CommandGroups.IntakeFromGround;
//import frc.robot.commands.GroundIntake.FlipGroundIntakeIn;
//import frc.robot.commands.GroundIntake.FlipGroundIntakeOut;
//import frc.robot.subsystems.Aligner;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  private final XboxController m_driver = new XboxController(0);
  private final XboxController m_gunner = new XboxController(1);

  private final Arm m_arm = new Arm();
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  //  new JoystickButton(m_gunner, Button.kA.value).whileTrue(new IntakeFromGround(m_groundIntake))
  //  new JoystickButton(m_gunner, Button.kY.value).whileTrue(new AlignCoral(m_aligner, .5));
    //new JoystickButton(m_driver, Button.kX.value).whileTrue(new SetArmSpeed(m_arm, 0.2));
    new Trigger(() -> m_gunner.getRightY() > 0.2).whileTrue(new SetArmSpeed(m_arm, 0.2));
    new Trigger(() -> m_gunner.getRightY() > 0.2).whileTrue(new SetArmSpeed(m_arm, -0.2));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
