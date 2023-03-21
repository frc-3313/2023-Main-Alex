
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Wrist extends SubsystemBase {
  private final CANSparkMax wristMotor = new CANSparkMax(Constants.WRIST_ID, MotorType.kBrushless);
  private final DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(Constants.WRIST_ENCODER_ID);

  private double kp = 0.01;
  private double minPowerAtExtended = 0.03;
  private final PIDController pid = new PIDController(kp, 0.0, 0.0);
  private double setpoint = Constants.STOW_WRIST_ANGLE;
  private double setpointIncrementer = 0.5;
  private double motorOutput = 0.0;
  private RelativeEncoder relEncoder;
  // Settings
  private boolean usingPID = true;
  private boolean settingMinLevel = true;
  private double wristSpeed = Constants.MAX_WRIST_SPEED;


  public Wrist() {
    pid.setTolerance(3.0);
    throughboreEncoder.setDistancePerRotation(360);
    wristMotor.setInverted(false);
    relEncoder = wristMotor.getEncoder();
    relEncoder.setPosition(0);
    wristMotor.setIdleMode(IdleMode.kBrake);
    setSetpoint(Constants.STOW_WRIST_ANGLE);
  }

  public void raise() {

    if (usingPID) {
      double tempSetpoint = setpoint += setpointIncrementer;
      setSetpoint(tempSetpoint);
    } else {
      motorOutput = 0.2;
    }
  }

  public void lower() {
    if (usingPID) {
      double tempSetpoint = setpoint -= setpointIncrementer;
      setSetpoint(tempSetpoint);
    } else {
      motorOutput = -0.2;
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
  public void setArmSpeed(double speed){
    wristSpeed = speed;
  }
  public double getPidOutput() {
    double speed = pid.calculate(getDegrees(), setpoint) + getFeedForward();
    if (speed >= wristSpeed) {
      return wristSpeed;
    }
    else if (speed <= -wristSpeed) {
      return -wristSpeed;
    }
    return speed;
  }

  public double getDegrees() {
    //return relEncoder.getPosition();
    // fix after encoder is installed
    return throughboreEncoder.getDistance();
  }

  public void setSetpoint(double newSetpoint) {
    if (newSetpoint > Constants.MAX_WRIST_ANGLE) {
      setpoint = Constants.MAX_WRIST_ANGLE;
    } else if (newSetpoint < Constants.MIN_WRIST_ANGLE) {
      setpoint = Constants.MIN_WRIST_ANGLE;
    } else {
      setpoint = newSetpoint;
    }
  }

  public boolean atSetpoint() {
    if((getDegrees() < setpoint + 2) && getDegrees() > setpoint - 2)
      return true;
    else
      return false;
    //return pid.atSetpoint();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("wrist encoder value", getDegrees());
    SmartDashboard.putNumber("wrist position error", pid.getPositionError());
    SmartDashboard.putNumber("Wrist PID Output", getPidOutput());
    SmartDashboard.putNumber("Wrist Setpoint", setpoint);
    SmartDashboard.putNumber("Wrist throughbore encoder value", throughboreEncoder.getDistance());

    if(!throughboreEncoder.isConnected()){
      Protect();
    }
    else
    if (usingPID) {
      wristMotor.set(getPidOutput());
    } else {
      if (getDegrees() >= Constants.MAX_WRIST_ANGLE) motorOutput = 0.0;
      if (getDegrees() <= Constants.MIN_WRIST_ANGLE) motorOutput = 0.0;
      if (settingMinLevel) motorOutput = minPowerAtExtended;
      wristMotor.set(motorOutput);
    }
  }
  public void Protect(){
      stop();
      wristMotor.setIdleMode(IdleMode.kBrake);
  }
  public CommandBase ProtectWristFactory() {
    return new RunCommand(() -> {Protect();}, this).withName("Protect Wrist");
  }
}
