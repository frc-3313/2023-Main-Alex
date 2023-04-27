
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(Constants.SHOULDER_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(Constants.SHOULDER_ID_2, MotorType.kBrushless);
  private final DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(Constants.SHOULDER_ENCODER_ID);
  
  private double kp = 0.01;
  private double minPowerAtExtended = 0.03;
  private final PIDController pid = new PIDController(kp, 0.0, 0.0);
  private double setpoint = Constants.STOW_ARM_ANGLE;
  private double setpointIncrementer = 2;
  private double motorOutput = 0.0;

  // Settings
  private boolean usingPID = true;
  private boolean settingMinLevel = true;
  public boolean restrictSpeed;

  public Arm() {
    pid.setTolerance(5.0);
    throughboreEncoder.setDistancePerRotation(360);
    leftMotor.setInverted(false);
    rightMotor.follow(leftMotor, true);
    leftMotor.setIdleMode(IdleMode.kBrake);
    setpoint = Constants.STOW_ARM_ANGLE;
  }

  public void raise() {

    if (usingPID) {
      double tempSetpoint = setpoint -= setpointIncrementer;
      setSetpoint(tempSetpoint);
    } else {
      motorOutput = Constants.ARM_SPEED;
    }
  }

  public void lower() {
    if (usingPID) {
      double tempSetpoint = setpoint += setpointIncrementer;
      setSetpoint(tempSetpoint);
    } else {
      motorOutput = -Constants.ARM_SPEED;
    }
  }

  public void stop() {
    if (!usingPID) {
      motorOutput = 0.0;
    }
  }

  public double getFeedForward() {
    return Math.cos(Math.toRadians(getDegrees()))*getMinPower();
  }

  public double getMinPower() {
    return minPowerAtExtended;
  }

  public double getPidOutput() {
    //System.out.println("arm setpoint " + setpoint);
    double speed = pid.calculate(getDegrees(), setpoint) + getFeedForward();
    if (speed >= Constants.MAX_ARM_SPEED) {
      return Constants.MAX_ARM_SPEED;
    }
    else if (speed <= -Constants.MAX_ARM_SPEED) {
      return -Constants.MAX_ARM_SPEED;
    }
    return speed;
  }

  public double getDegrees() {
    return throughboreEncoder.getDistance();
  }

  public void setSetpoint(double newSetpoint) {
    if(!restrictSpeed){
    if (newSetpoint > Constants.MAX_ARM_ANGLE) {
      setpoint = Constants.MAX_ARM_ANGLE;
    } else if (newSetpoint < Constants.MIN_ARM_ANGLE) {
      setpoint = Constants.MIN_ARM_ANGLE;
    } else {
      setpoint = newSetpoint;
    }
    }
    else{
      setpoint = getDegrees();
    }

  }

  public boolean atSetpoint() {
    //return pid.atSetpoint();
    if((getDegrees() < setpoint + 5) && getDegrees() > setpoint-5)
      return true;
    else
      return false;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm throughbore encoder value", getDegrees());
    SmartDashboard.putNumber("arm position error", pid.getPositionError());
    SmartDashboard.putNumber("Arm PID Outpoint", getPidOutput());
    SmartDashboard.putNumber("Arm Setpoint", setpoint);
    if(!throughboreEncoder.isConnected()){
      stop();
      leftMotor.setIdleMode(IdleMode.kBrake);
    }
    else if (usingPID) {
      //System.out.println("Arm Speed PID :" + getPidOutput());
      leftMotor.set(getPidOutput());
    } else {
      if (getDegrees() >= Constants.MAX_ARM_ANGLE) motorOutput = 0.0;
      if (getDegrees() <= Constants.MIN_ARM_ANGLE) motorOutput = 0.0;
      if (settingMinLevel) motorOutput = minPowerAtExtended;
      leftMotor.set(motorOutput);
    }
  }
}