// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Grabber extends SubsystemBase {

  private final Talon m_roller = new Talon(Constants.GRABBER_ROLLER_ID);
  DoubleSolenoid grabberSolenoid = new DoubleSolenoid(Constants.GRABBER_CAN, PneumaticsModuleType.CTREPCM, Constants.GRABBER_DROP, Constants.GRABBER_GRAB);

  public boolean hasGamePiece;
  public void grabPiece() {
    grabberSolenoid.set(Value.kReverse);
    hasGamePiece = true;
    //stopRollers();
  }
  
  public void dropPiece() {
    grabberSolenoid.set(Value.kForward);
    hasGamePiece = false;
  }
  
  public void startRollers() {
    m_roller.set(-.5);
  }

  public void startReverseRollers() {
    m_roller.set(.5);
  }

  public void stopRollers() {
    m_roller.set(0);
  }

  // COMMAND FACTORIES

  public CommandBase grabPieceFactory() {
    return new RunCommand(() -> {grabPiece();}, this).withName("Grab Piece");
  }

  public CommandBase dropPieceFactory() {
    return new RunCommand(() -> {dropPiece(); }, this).withName("Drop Piece");
  }

  public CommandBase startRollersCommand() {
    return new RunCommand(() -> {startRollers();}, this).withName("Rollers Start");
  }

  public CommandBase startRollersReverseCommand() {
    return new RunCommand(() -> {startReverseRollers();}, this).withName("Reverse Rollers Start");
  }

  public CommandBase stopRollersCommand() {
    return new RunCommand(() -> {stopRollers();}, this).withName("Rollers Stop");
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Grabber Closed", hasGamePiece);
  }
}
