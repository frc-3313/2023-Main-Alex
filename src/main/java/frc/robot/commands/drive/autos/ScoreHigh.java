package frc.robot.commands.drive.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ArmWristCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveDistanceMeters;

public class ScoreHigh {
    public static SequentialCommandGroup ScoreHighCommand(Swerve s_Swerve, Arm s_Arm, Wrist s_Wrist, Grabber s_Grabber){
        
        return new SequentialCommandGroup(
            new ArmWristCommand(s_Arm, Constants.HIGH_ARM_ANGLE, Constants.AUTO_ARM_SPEED, s_Wrist, Constants.HIGH_WRIST_ANGLE, Constants.AUTO_WRIST_SPEED).withTimeout(3),
            new DriveDistanceMeters(s_Swerve, 1, .1),
            new WaitCommand(.25),
            new AutoOpenGrabber(s_Grabber).withTimeout(.5),
            new DriveDistanceMeters(s_Swerve, -.5, .1),
            new ArmWristCommand(s_Arm, Constants.STOW_ARM_ANGLE, Constants.AUTO_ARM_SPEED, s_Wrist, Constants.STOW_WRIST_ANGLE, Constants.AUTO_WRIST_SPEED).withTimeout(0.5)
        );
    }

}