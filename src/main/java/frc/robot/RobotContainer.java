package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.controllers.ButtonBox;
import frc.robot.commands.autos.AutoHighR;
import frc.robot.commands.autos.MeterPlace;
import frc.robot.commands.drive.DriveByController;
import frc.robot.commands.ArmWristCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    SendableChooser<Command> auto_chooser = new SendableChooser<>();
    //public double accel;
    //public double accel2;
    /* Controllers */
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final ButtonBox m_buttonBox = new ButtonBox(1);

    /* Driver Buttons */

    /* Subsystems */
    private final Drivetrain s_Swerve = new Drivetrain();
    private final Grabber m_grabber = new Grabber();
    private final Arm m_arm = new Arm();
    private final Wrist m_wrist = new Wrist();
    public final Timer m_timer = new Timer();
    public double gyroOffset = 0.0;

    /* Autos */
    private final Command auto1 = new AutoHighR(s_Swerve, m_arm, m_wrist, m_grabber, m_timer);
    private final Command Auto1Meter = new MeterPlace(s_Swerve, m_arm, m_wrist, m_grabber, m_timer);
    private final Command WaitHere = new WaitCommand(1);

    /* Commands */
    private final DriveByController m_drive = new DriveByController(s_Swerve, m_driverController);
    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {     
        
        s_Swerve.setDefaultCommand(m_drive);


        // Configure the button bindings
        configureButtonBindings();

        auto_chooser.setDefaultOption("Do Nothing", WaitHere);
        auto_chooser.addOption("Drive 1 Meter", Auto1Meter);
        auto_chooser.addOption("One Piece High and back", auto1);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        new POVButton(m_driverController, 0).onTrue(s_Swerve.resetOdometryFactory(new Rotation2d(0.0)));
    // ARM (CONTROL STICK)
    m_buttonBox.ArmUp.whileTrue(new RunCommand(() -> m_wrist.raise())).onFalse(new InstantCommand(() -> m_arm.stop()));
    m_buttonBox.ArmDown.whileTrue(new RunCommand(() -> m_wrist.lower())).onFalse(new InstantCommand(() -> m_arm.stop()));
    // SET LOW
    m_buttonBox.setLow.onTrue(new ArmWristCommand(m_arm, Constants.LOW_ARM_ANGLE, m_wrist, Constants.LOW_WRIST_ANGLE));

    // SET MID
    m_buttonBox.setMiddle.onTrue(new ArmWristCommand(m_arm, Constants.MID_ARM_ANGLE, m_wrist, Constants.MID_WRIST_ANGLE));

    // SET HIGH
    m_buttonBox.setHigh.onTrue(new ArmWristCommand(m_arm, Constants.HIGH_ARM_ANGLE, m_wrist, Constants.HIGH_WRIST_ANGLE));
    
    // SET stow
    m_buttonBox.setStow.onTrue(new ArmWristCommand(m_arm, Constants.STOW_ARM_ANGLE, m_wrist, Constants.STOW_WRIST_ANGLE));
    

    //grabber commands  
    m_buttonBox.grabButton.onTrue(m_grabber.grabPieceFactory());
    m_buttonBox.dropButton.onTrue(m_grabber.dropPieceFactory());
    m_buttonBox.DownDpadTrigger.onTrue(m_grabber.startRollersCommand()).onFalse(m_grabber.stopRollersCommand());
    m_buttonBox.UpDpadTrigger.onTrue(m_grabber.startRollersReverseCommand()).onFalse(m_grabber.stopRollersCommand());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return auto1;
        //return auto_chooser.getSelected();
        return Auto1Meter;
    }
}
