package frc.robot;

import javax.management.InstanceNotFoundException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.States.intakeState;
import frc.robot.States.AmpEnums.ampArmSetpoints;
import frc.robot.States.AmpEnums.ampIndexState;
import frc.robot.States.shooterEnums.aimingSetPoints;
import frc.robot.States.shooterEnums.feederState;
import frc.robot.States.shooterEnums.shooterState;
import frc.robot.autos.*;
import frc.robot.CommandFactory.AmpFactory;
import frc.robot.CommandFactory.ShooterFactory;
import frc.robot.commands.*;
import frc.robot.commands.PhotonVisionCmds.RotateMove;
import frc.robot.subsystems.*;
//import frc.robot.subsystems.SysIDTest;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    // private final Joystick driver = new Joystick(0);
    private final CommandXboxController driver0 = new CommandXboxController(0);
    private final CommandXboxController driver1 = new CommandXboxController(1);

    /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;
    
    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ShooterSubsystem s_shooter = new ShooterSubsystem();
    private final AmpSubsystem s_amp = new AmpSubsystem();
    private final IntakeSubsystem s_intake = new IntakeSubsystem();
    private final SysIDTest testingSwerve = new SysIDTest(s_Swerve);
  //  private final PhotonVision camera1 = new PhotonVision(null);
   // private final SysIDTest testingSwerve = new SysIDTest(s_Swerve);

   //Commands
   private final AmpFactory ampFactory = new CommandFactory.AmpFactory(s_amp);
   private final ShooterFactory shooterFactory = new CommandFactory.ShooterFactory(s_shooter);

    //private final RotateMove c_DriveToTag = new RotateMove(null,null,null,null,null);
    // aim amp arms
    private final AmpCommand c_ampHome = ampFactory.createAmpHomeCommand();
    private final AmpCommand c_ampScore = ampFactory.createAmpScoreCommand();

    // score * intake notes 
    private final AmpCommand c_ampIndexIn = ampFactory.createAmpIndexInCommand();
    private final AmpCommand c_ampIndexOut = ampFactory.createAmpIndexOutCommand();

    // feed notes into the shooter
    private final ShooterCommand c_shooterFeedIn = shooterFactory.createIntakeCommand();
    private final ShooterCommand c_shooterFeedOut = shooterFactory.createFeedOutCommand();
    private final ShooterCommand c_shooterFeedStop = shooterFactory.createFeedStopCommand();

    // shoot the notes
    private final ShooterCommand c_ShooterShoot = shooterFactory.createShootCommand();
    private final ShooterCommand c_shooterUnshoot = shooterFactory.createUnshootCommand();
    private final ShooterCommand c_shooterStop = shooterFactory.createStopShootCommand();

    // aim the shooter
    private final ShooterCommand c_shooterAimNear = shooterFactory.createAimNearCommand();
    private final ShooterCommand c_shooterAimFar = shooterFactory.createAimFarCommand();
    private final ShooterCommand c_shooterAimForIntake = shooterFactory.createAimForIntakeCommand();
    private final ShooterCommand c_shooterAimHome = shooterFactory.createShooterHomeCommand();

    private final IntakeCommand c_intakeIn = new IntakeCommand(s_intake, intakeState.INTAKE);
    private final IntakeCommand c_intakeOut = new IntakeCommand(s_intake, intakeState.OUT);
    private final IntakeCommand c_intakeStop = new IntakeCommand(s_intake, intakeState.STOP);
    private final IntakeCommand c_intakeToAmp = new IntakeCommand(s_intake, intakeState.AMP_INTAKE);
    private final IntakeCommand c_intakeAmpOut = new IntakeCommand(s_intake, intakeState.OUT);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // s_Swerve.setDefaultCommand(
        //     new TeleopSwerve(
        //         s_Swerve, 
        //         () -> -driver.getRawAxis(translationAxis), 
        //         () -> -driver.getRawAxis(strafeAxis), 
        //         () -> -driver.getRawAxis(rotationAxis), 
        //         () -> robotCentric.getAsBoolean()
        //     )
        // );

        s_Swerve.setDefaultCommand(new TeleopSwerve(
            s_Swerve, 
            () -> -driver0.getLeftY(), 
            () -> -driver0.getLeftX(), 
            () -> -driver0.getRightX(),
            () -> driver0.y().getAsBoolean()));
        // Configure the button bindings

        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver0.rightBumper().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //configureTestBindings();
        configureDriver1AmpBindings();
        configureDriver1IntakeBindings();
        configureDriver1ShooterBindings();
    }

    private void configureTestBindings() {
        driver1.a().onTrue(testingSwerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driver1.b().onTrue(testingSwerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driver1.x().onTrue(testingSwerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driver1.y().onTrue(testingSwerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    }

    private void configureDriver1ShooterBindings() {
        driver1.pov(0).onTrue(c_shooterAimFar);
        driver1.pov(90).onTrue(c_shooterAimNear);
        // unessaery change later
        driver1.pov(180).onTrue(c_shooterAimForIntake);
        driver1.pov(-1).onTrue(c_shooterAimHome);

        driver1.rightTrigger().onTrue(c_ShooterShoot);
        driver1.rightTrigger().onFalse(c_shooterStop);

        driver1.rightBumper().onTrue(c_shooterFeedIn);
        driver1.rightBumper().onFalse(c_shooterFeedStop);

        driver1.pov(270).onTrue(c_shooterUnshoot);
        driver1.pov(270).onTrue(c_shooterFeedOut);
        driver1.pov(270).onFalse(c_shooterFeedStop);
        driver1.pov(270).onFalse(c_shooterStop);
    }

    private void configureDriver1AmpBindings() {
        driver1.leftTrigger().onTrue(c_ampScore);
        driver1.leftTrigger().onFalse(c_ampHome);

        driver1.leftBumper().onTrue(c_ampIndexIn);
        driver1.leftBumper().onFalse(c_ampIndexOut);
    }

    private void configureDriver1IntakeBindings() {
        driver1.a().onTrue(c_intakeIn);
        driver1.a().onFalse(c_intakeStop);

        driver1.b().onTrue(c_intakeOut);
        driver1.b().onFalse(c_intakeStop);

        driver1.x().onTrue(c_intakeToAmp);
        driver1.x().onFalse(c_intakeStop);
        
        driver1.y().onTrue(c_intakeAmpOut);
        driver1.y().onFalse(c_intakeStop);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
