package frc.robot.util.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverController {
    
    public double getTranslationX();

    public double getTranslationY();

    public double getRotation();

    public boolean getGyroReset();

    public Trigger getShootButton();

    public boolean getLeftBumper();

    public boolean getRightBumper();

}