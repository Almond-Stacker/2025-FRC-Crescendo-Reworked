package frc.robot;

import frc.robot.commands.AmpCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.*;
import frc.robot.States.shooterEnums.*;
import frc.robot.States.AmpEnums.*;

public class commandFactory {
    
    public class shooterFactory {
        private ShooterSubsystem s_shooter;

        public shooterFactory(ShooterSubsystem shooter) {
            this.s_shooter = shooter;
        }

        public final ShooterCommand c_shooterFeedIn = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
            .withFeederState(feederState.INTAKE).build());
        public final ShooterCommand c_shooterFeedOut = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
            .withFeederState(feederState.OUT).build());
        public final ShooterCommand c_shooterShoot = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
            .withShooterState(shooterState.SHOOT).build());
        public final ShooterCommand c_shooterUnshoot = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
            .withShooterState(shooterState.FEEDBACK).build());
        public final ShooterCommand c_shooterStop = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
            .withShooterState(shooterState.STOP).build());
        public final ShooterCommand c_shooterAimNear = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
            .withAimState(aimingSetPoints.NEAR).build());
        public final ShooterCommand c_shooterAimFar = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
            .withAimState(aimingSetPoints.FAR).build());
        public final ShooterCommand c_shooterAimForIntake = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
            .withAimState(aimingSetPoints.INTAKE).build());
        public final ShooterCommand c_shooterHome = new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter)
            .withAimState(aimingSetPoints.HOME).build());
    }

    public class ampFactory {
        private AmpSubsystem s_amp;

        public ampFactory(AmpSubsystem amp) {
            s_amp = amp;
        }

        public final AmpCommand c_ampHome = new AmpCommand(new AmpCommand.AmpConfiguration(s_amp)
            .withAimingSetPoints(ampArmSetpoints.HOME).build());
        public final AmpCommand c_ampScore = new AmpCommand(new AmpCommand.AmpConfiguration(s_amp)
            .withAimingSetPoints(ampArmSetpoints.TRAP).build());
        public final AmpCommand c_ampIndexIn = new AmpCommand(new AmpCommand.AmpConfiguration(s_amp)
            .withIndexState(ampIndexState.INTAKE).build());
        public final AmpCommand c_ampIndexOut = new AmpCommand(new AmpCommand.AmpConfiguration(s_amp)
            .withIndexState(ampIndexState.OUT).build());
    }

}
