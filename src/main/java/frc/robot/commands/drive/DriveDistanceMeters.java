// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveDistanceMeters extends CommandBase {
  private Swerve m_drive;
  private Pose2d startPose;
  private double distanceMeters;
  private double translationVelocityMetersPerSecond;
  /** Creates a new DriveDistanceMeters. */
  public DriveDistanceMeters(Swerve drive, double distanceMeters, double translationVelocityMetersPerSecond) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    this.distanceMeters = distanceMeters;
    this.translationVelocityMetersPerSecond = Math.abs(translationVelocityMetersPerSecond);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPose = m_drive.getPose();
    if (distanceMeters < 0.0 ) {
      translationVelocityMetersPerSecond *= -1.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drive.drive(new Translation2d(this.translationVelocityMetersPerSecond, 0.0), 0.0,false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = m_drive.getPose();
    Pose2d relativePose = currentPose.relativeTo(startPose);
    boolean shouldStop = false;
    if (distanceMeters >= 0.0 ) {
        shouldStop = relativePose.getX() > distanceMeters;
    }
    else {
        shouldStop = relativePose.getX() < distanceMeters;
    }
    return shouldStop;
  }
}