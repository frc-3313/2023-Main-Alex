// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class FollowHolonomicTrajectory extends CommandBase {
  private final Swerve m_drive;
  private final Trajectory m_trajectory;
  private final Rotation2d m_rotation;
  private final boolean m_resetOdometry;
  private final Timer m_timer = new Timer();

  private final static double p_thetaMaxVelocity = Math.PI;
  private final static double p_thetaMaxAcceleration = Math.PI;
  private final static double p_xTolerance = 0.1;
  private final static double p_yTolerance = 0.1;
  private final static double p_thetaTolerance = 0.05;

  private HolonomicDriveController m_controller;
  private boolean m_cancel = false;

  /** Creates a new FollowHolonomicTrajectory. */
  public FollowHolonomicTrajectory(Swerve drive, Trajectory trajectory, Rotation2d startRotation,  Rotation2d endRotation, boolean resetOdometry) {
    m_drive = drive;
    m_trajectory = trajectory;
    m_rotation = endRotation.minus(startRotation);
    m_resetOdometry = resetOdometry;

    addRequirements(m_drive);
  }

  public FollowHolonomicTrajectory(Swerve drive, Trajectory trajectory, boolean resetOdometry) {
    this(drive, trajectory, new Rotation2d(), new Rotation2d(), resetOdometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cancel = false;

    m_controller = new HolonomicDriveController(
      new PIDController(0.0, 0.0, 0.0),
      new PIDController(0.0, 0.0, 0.0),
      new ProfiledPIDController(0.0, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(p_thetaMaxVelocity, p_thetaMaxAcceleration)));

    m_controller.setTolerance(new Pose2d(p_xTolerance, p_yTolerance, Rotation2d.fromDegrees(p_thetaTolerance)));

    if (m_resetOdometry) {
      m_drive.resetTrajectoryPose(m_trajectory.getInitialPose());
    } else {
      Transform2d offset = m_drive.getPose().minus(m_trajectory.getInitialPose());
      if (offset.getTranslation().getDistance(new Translation2d()) > 3.0 || offset.getRotation().getDegrees() > 45.0 ) {
        System.out.println("!!!canceling holomic trajectory!!!");
        m_cancel = true;
      }
    }

    m_controller.setEnabled(true);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();
    Trajectory.State desiredState = m_trajectory.sample(m_timer.get());
    Rotation2d targetRotation = new Rotation2d(m_timer.get() * m_rotation.getRadians() / m_trajectory.getTotalTimeSeconds());

    ChassisSpeeds targetSpeeds = m_controller.calculate(currentPose, desiredState, targetRotation);

    SmartDashboard.putNumber("Auto:Measured vX", m_drive.getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Auto:Measured vY", m_drive.getChassisSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Auto:Measured omega", m_drive.getChassisSpeeds().omegaRadiansPerSecond);

    SmartDashboard.putNumber("Auto:Desired vX", desiredState.velocityMetersPerSecond * Math.cos(desiredState.poseMeters.getRotation().getRadians()));
    SmartDashboard.putNumber("Auto:Desired vY", desiredState.velocityMetersPerSecond * Math.sin(desiredState.poseMeters.getRotation().getRadians()));
    SmartDashboard.putNumber("Auto:Desired omega", desiredState.curvatureRadPerMeter);

    SmartDashboard.putNumber("Auto:Commanded vX", targetSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Auto:Commanded vY", targetSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Auto:Commanded omega", targetSpeeds.omegaRadiansPerSecond);

    if (!m_cancel) m_drive.drive(targetSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  m_timer.get() > m_trajectory.getTotalTimeSeconds() || m_cancel;
  }
}
