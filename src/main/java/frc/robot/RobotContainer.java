package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.controllers.ButtonBox;
import frc.robot.commands.autos.AutoHighR;
import frc.robot.commands.autos.Drive1Meter;
import frc.robot.commands.autos.MeterPlace;

import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ArmWristCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

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
    private final Joystick driver = new Joystick(0);
    private final ButtonBox m_buttonBox = new ButtonBox(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int speedControl = XboxController.Axis.kRightTrigger.value;
    /* Driver Buttons */
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Grabber m_grabber = new Grabber();
    private final Arm m_arm = new Arm();
    private final Wrist m_wrist = new Wrist();
    public final Timer m_timer = new Timer();
    public double gyroOffset = 0.0;

    /* Autos */
    private final Command auto1 = new AutoHighR(s_Swerve, m_arm, m_wrist, m_grabber, m_timer);
    private final Command Auto1Meter = new MeterPlace(s_Swerve, m_arm, m_wrist, m_grabber, m_timer);
    private final Command WaitHere = new WaitCommand(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {     
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis)/1.3, 
                () -> -driver.getRawAxis(strafeAxis)/1.3, 
                () -> driver.getRawAxis(rotationAxis)/2/9*13.5,    ///2/9*6.75, 
                () -> robotCentric.getAsBoolean(), 
                () -> driver.getRawAxis(speedControl)/1.5,
                () -> gyroOffset
            )
        );

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
        //zeroGyro.onTrue(new InstantCommand(() -> this.zeroGyro()));
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

    public void zeroGyro() {
        s_Swerve.zeroGyro();
        gyroOffset = 0.0;
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
