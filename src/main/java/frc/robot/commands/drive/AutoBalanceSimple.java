// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalanceSimple extends CommandBase {
  /** Creates a new AutoBalanceSimple. */
  private Swerve m_drive;
  private Pose2d m_startPose;
  private double m_lastAngle;
  private int m_lockedCounter;
  private int m_driveCounter;
  private boolean m_driving= false;
  private double m_climbSpeed;

  private final SwerveModuleState [] LOCK_STATES = { 
    new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(90))
  };

  private final double p_rollOffset = 0.0;
  private final double p_levelThreshold = 4.0;
  private final double p_movingThreshold = 0.2;
  private final int p_lockMin = 10;
  private final double p_initialClimbSpeed = 0.5;
  private final double p_climbMaxDistance = 1.5;

  public AutoBalanceSimple(Swerve drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startPose = m_drive.getPose();
    m_lastAngle = getChargeStationAngle();
    m_climbSpeed = p_initialClimbSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = getChargeStationAngle();
    double deltaAngle = currentAngle - m_lastAngle;
    m_lastAngle = currentAngle;
    
    SmartDashboard.putNumber("Auto:currentAngle", currentAngle);
    SmartDashboard.putNumber("Auto:deltaAngle", deltaAngle);
    SmartDashboard.putNumber("Auto:climbSpeed", m_climbSpeed);

    if (m_driving) m_driveCounter++;

    if (Math.abs(currentAngle) < p_levelThreshold) {
      // if the angle is level, lock in place
      SmartDashboard.putNumber("Auto:condition", 0);
      m_lockedCounter = 0;
      lock();
    } else if (Math.abs(deltaAngle) > p_movingThreshold && (m_driving && m_driveCounter > 15 || !m_driving)) {
      // if the angle is moving, lock in place
      SmartDashboard.putNumber("Auto:condition", 1);
      m_lockedCounter = 0;
      lock();
    } else if (m_drive.getPose().getTranslation().getDistance(m_startPose.getTranslation()) > p_climbMaxDistance) {
      // if we have driven too far from where we began, lock in place
      SmartDashboard.putNumber("Auto:condition", 2);
      m_lockedCounter = 0;
      lock();
    } else if (m_lockedCounter< p_lockMin) {
      // if we recently locked, stay locked
      m_lockedCounter++;
      lock();
    } else {
      // drive up
      m_drive.drive(m_climbSpeed * Math.signum(currentAngle), 0, 0);
      if (!m_driving) {
        m_driving = true;
        m_driveCounter = 0;
        m_climbSpeed = m_climbSpeed / 2.00;

        if (m_climbSpeed <= 0.5) {
          m_climbSpeed = 0.5;
        }
      }
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

  private double getChargeStationAngle(){
    // double yaw = m_drive.getGyroscopeRotation().getDegrees();
    // double pitch = m_drive.getNavX().getPitch();
    // double roll = m_drive.getNavX().getRoll();
    // return (pitch*-Math.cos(Math.toRadians(yaw)))+(roll*Math.sin(Math.toRadians(yaw)));
    // trying something simple first:
    return m_drive.getRoll() - p_rollOffset;
  }

  private void lock() {
    m_driving =  false;
    m_drive.setModuleStates(LOCK_STATES);
  }
}
