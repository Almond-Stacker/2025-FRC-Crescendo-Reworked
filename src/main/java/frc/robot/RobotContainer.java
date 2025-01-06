// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandFactory.AmpFactory;
import frc.robot.CommandFactory.ShooterFactory;
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
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController driver1 = new CommandXboxController(1); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final ShooterSubsystem s_shooter = new ShooterSubsystem();
  private final AmpSubsystem s_amp = new AmpSubsystem();
  private final IntakeSubsystem s_intake = new IntakeSubsystem();

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


  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
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

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
