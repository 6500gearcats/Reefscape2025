// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class dpadAlign2 extends Command {
  /** Creates a new PathFindToPos. */
  PathConstraints constraints;
  DriveSubsystem m_drive;
  AprilTagFieldLayout field;
  boolean right;
  public dpadAlign2(DriveSubsystem newM_Drive) {
    m_drive = newM_Drive;
    constraints = new PathConstraints(1.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    addRequirements(m_drive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (LimelightHelpers.getTV("limelight-gcc")) {
      // Command pathFindCommand = AutoBuilder.pathfindToPose(//getPoseOffset(
      // getBestAprilTag(), constraints);
      // pathFindCommand.schedule();

      m_drive.drive()
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d myPose = m_drive.getPose();
    
    m_drive.drive()
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, interrupted);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getPose().equals(getBestAprilTag());
  }

  // private Pose2d getPoseOffset(Pose2d pose) {

  // }

  private Pose2d getBestAprilTag() {
    field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    Pose2d pose = m_drive.getPose();
    int bestAprilTag = getClosestAprilTagID(pose.getTranslation());
    Pose2d newPose = field.getTagPose(bestAprilTag).get().toPose2d();
    System.out.println("Old Poses values" + newPose.getX() + ", " + newPose.getY() + ". Rotation: " + newPose.getRotation());

    double tempAngle = field.getTagPose(bestAprilTag).get().toPose2d().getRotation().getRadians();
    double newX = 0;
    double newY = 0;
    newX = (newPose.getX() + Math.cos(tempAngle) * .66) + Math.cos(tempAngle + Math.PI/2) * .3;
    newY = (newPose.getY() + Math.sin(tempAngle) * .66) + Math.sin(tempAngle + Math.PI/2) * .3;
    Pose2d thirdPose = new Pose2d(newX, newY, newPose.getRotation());
    //.getRotation().plus(new Rotation2d(Math.PI)));
    System.out.println("New Poses values" + thirdPose.getX() + ", " + thirdPose.getY() + ". Rotation: " + thirdPose.getRotation());
    return thirdPose;
  }

  private int getClosestAprilTagID(Translation2d robotPose) {
    int integer = 0;
    ArrayList<Double> poses = new ArrayList<Double>();
    ArrayList<Integer> list = new ArrayList<Integer>();
    for (int i = 1; i<23; i++) {
      Translation2d pose = field.getTagPose(i).get().getTranslation().toTranslation2d();
      double distance = robotPose.getDistance(pose);
      poses.add(distance);
      list.add(i);
    }
    double minValue = Collections.min(poses);
    System.out.println("Min value: " + minValue);
    integer = poses.indexOf(minValue);
    System.out.println("Integer of least distance: " + integer);
    return list.get(integer);
  }
}



