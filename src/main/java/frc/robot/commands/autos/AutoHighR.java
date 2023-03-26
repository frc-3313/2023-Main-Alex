package frc.robot.commands.autos;

import frc.robot.Constants;
import frc.robot.commands.AutoSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ArmWristCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoHighR extends SequentialCommandGroup {
    public AutoHighR(Drivetrain s_Swerve, Arm s_Arm, Wrist s_wrist, Grabber s_Grabber,Timer m_timer){

        Command stopswervecommand = new AutoSpeed(s_Swerve, 0, 0, 0, 0, m_timer);
        Command setArmHigh = new ArmWristCommand(s_Arm, Constants.HIGH_ARM_ANGLE, s_wrist, Constants.HIGH_WRIST_ANGLE);
        Command setArmStow = new ArmWristCommand(s_Arm, Constants.STOW_ARM_ANGLE, s_wrist, Constants.STOW_WRIST_ANGLE);
        Command dropPiececommand = new AutoOpenGrabber(s_Grabber, 0.2, m_timer);

        Command as1command = new AutoSpeed(s_Swerve, -0.32, 0.0, 0.0, 2, m_timer);
        Command as2command = new AutoSpeed(s_Swerve, 0.5, 0, 0, 2, m_timer);
        Command as3command = new AutoSpeed(s_Swerve, 3.3, 0, 0, 5, m_timer);


        addCommands(
            setArmHigh,
            as1command,
            dropPiececommand,
            as2command,
            setArmStow,
            as3command,
            stopswervecommand

        );
    }
}