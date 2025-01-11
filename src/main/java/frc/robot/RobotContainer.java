// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.CommandFactory.AmpFactory;
import frc.robot.CommandFactory.ShooterFactory;
import frc.robot.Constants;
import frc.robot.States.intakeState;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    // Initalize Subsytems and subsystem controllers 
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driver0 = new CommandXboxController(0);
    private final CommandXboxController driver1 = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

     private final ShooterSubsystem s_shooter = new ShooterSubsystem();
    private final AmpSubsystem s_amp = new AmpSubsystem();
    private final IntakeSubsystem s_intake = new IntakeSubsystem();

    /* Path follower */
    // Choreo set up 
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

       //Commands
    private final AmpFactory ampFactory = new CommandFactory.AmpFactory(s_amp);
    private final ShooterFactory shooterFactory = new CommandFactory.ShooterFactory(s_shooter);

     // aim amp arms
    private final AmpCommand c_ampHome = ampFactory.createAmpHomeCommand();
    private final AmpCommand c_ampScore = ampFactory.createAmpScoreCommand();

    // score * intake notes 
    private final AmpCommand c_ampIndexIn = ampFactory.createAmpIndexInCommand();
    private final AmpCommand c_ampIndexOut = ampFactory.createAmpIndexOutCommand();
    private final AmpCommand c_ampIndexStop = ampFactory.createAmpIndexStopCommand();

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

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureDriveBindings();
    }

    private void configureDriveBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            //Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver0.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver0.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver0.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        driver0.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver0.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))));

        driver0.pov(0).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0.5).withVelocityY(0))
        );
        driver0.pov(180).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver0.back().and(driver0.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver0.back().and(driver0.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver0.start().and(driver0.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver0.start().and(driver0.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        configureDriver1AmpBindings();
        configureDriver1IntakeBindings();
        configureDriver1ShooterBindings();
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
    
        driver1.leftBumper().onTrue(c_ampIndexOut);
        driver1.leftBumper().onFalse(c_ampIndexStop);
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
        driver1.y().onTrue(c_ampIndexIn);
        driver1.y().onFalse(c_ampIndexStop);
      }
    
    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}