package frc.robot;

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
    private final Joystick driver = new Joystick(0);
    private final CommandXboxController driver2 = new CommandXboxController(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ShooterSubsystem s_shooter = new ShooterSubsystem();
    private final AmpSubsystem s_amp = new AmpSubsystem();
    private final IntakeSubsystem s_intake = new IntakeSubsystem();
    private final SysIDTest testingSwerve = new SysIDTest(s_Swerve);
  //  private final PhotonVision camera1 = new PhotonVision(null);
   // private final SysIDTest testingSwerve = new SysIDTest(s_Swerve);

   //Commands
   //private final RotateMove c_DriveToTag = new RotateMove(null,null,null,null,null);
    private final AmpCommand c_ampHome = new AmpCommand(new AmpCommand.AmpConfiguration(s_amp)
        .withAimingSetPoints(ampArmSetpoints.HOME).build());
    private final AmpCommand c_ampScore = new AmpCommand(new AmpCommand.AmpConfiguration(s_amp)
        .withAimingSetPoints(ampArmSetpoints.TRAP).build());
    private final AmpCommand c_ampIndexIn = new AmpCommand(new AmpCommand.AmpConfiguration(s_amp)
        .withIndexState(ampIndexState.INTAKE).build());
    private final AmpCommand c_ampIndexOut = new AmpCommand(new AmpCommand.AmpConfiguration(s_amp)
        .withIndexState(ampIndexState.OUT).build());

    private final ShooterCommand c_shooterFeedIn = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
        .withFeederState(feederState.INTAKE).build());
    private final ShooterCommand c_shooterFeedOut = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
        .withFeederState(feederState.OUT).build());
    private final ShooterCommand c_shooterShoot = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
        .withShooterState(shooterState.SHOOT).build());
    private final ShooterCommand c_shooterUnshoot = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
        .withShooterState(shooterState.FEEDBACK).build());
    private final ShooterCommand c_shooterStop = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
        .withShooterState(shooterState.STOP).build());
    private final ShooterCommand c_shootAimNear = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
        .withAimState(aimingSetPoints.NEAR).build());
    private final ShooterCommand c_shootAimFar = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
        .withAimState(aimingSetPoints.FAR).build());


    private final IntakeCommand c_intakeIn = new IntakeCommand(s_intake, intakeState.INTAKE);
    private final IntakeCommand c_intakeOut = new IntakeCommand(s_intake, intakeState.OUT);
    private final IntakeCommand c_intakeStop = new IntakeCommand(s_intake, intakeState.STOP);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //configureTestBindings();
    }

    private void configureTestBindings() {
        driver2.a().onTrue(testingSwerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driver2.b().onTrue(testingSwerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driver2.x().onTrue(testingSwerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driver2.y().onTrue(testingSwerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
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
