package frc.robot.commands.drive.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ArmWristCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveDistanceMeters;

public class GroundPickup {
    public static SequentialCommandGroup GroundPickupCommand(Swerve s_Swerve, Arm s_Arm, Wrist s_Wrist, Grabber s_Grabber){
        
        return new SequentialCommandGroup(
            s_Grabber.startRollersCommand().withTimeout(.25),
            new DriveDistanceMeters(s_Swerve, .25, .5),
            s_Grabber.grabPieceFactory().withTimeout(0.5),
            s_Grabber.stopRollersCommand().withTimeout(.1),
            new ArmWristCommand(s_Arm, Constants.STOW_ARM_ANGLE, Constants.AUTO_ARM_SPEED, s_Wrist, Constants.STOW_WRIST_ANGLE, Constants.AUTO_WRIST_SPEED).withTimeout(0.5)
        );
    }

}