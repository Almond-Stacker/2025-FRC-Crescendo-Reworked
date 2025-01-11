package frc.robot;

import frc.robot.commands.AmpCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.*;
import frc.robot.States.shooterEnums.*;
import frc.robot.States.AmpEnums.*;

public class CommandFactory {
    
    public static class ShooterFactory {
        private ShooterSubsystem s_shooter;

        public ShooterFactory(ShooterSubsystem shooter) {
            this.s_shooter = shooter;
        }

        public ShooterCommand createIntakeCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withFeederState(feederState.INTAKE).build());
        }
        public ShooterCommand createFeedOutCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withFeederState(feederState.OUT).build());
        }
        public ShooterCommand createFeedStopCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withFeederState(feederState.STOP).build());
        }
        public ShooterCommand createShootCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withShooterState(shooterState.SHOOT).build());
        }
        public ShooterCommand createUnshootCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withShooterState(shooterState.FEEDBACK).build());
        }
        public ShooterCommand createStopShootCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withShooterState(shooterState.STOP).build());
        }
        public ShooterCommand createAimNearCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withAimState(aimingSetPoints.NEAR).build());
        }
        public ShooterCommand createAimFarCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withAimState(aimingSetPoints.FAR).build());
        }
        public ShooterCommand createAimForIntakeCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withAimState(aimingSetPoints.INTAKE).build());
        }
        public ShooterCommand createShooterHomeCommand() {
            return new ShooterCommand(new ShooterCommand.ShooterConfiguration(s_shooter).withAimState(aimingSetPoints.HOME).build());
        }
    }

    public static class AmpFactory {
        private AmpSubsystem s_amp;

        public AmpFactory(AmpSubsystem amp) {
            s_amp = amp;
        }

        public AmpCommand createAmpHomeCommand() {
            return new AmpCommand(new AmpCommand.AmpConfiguration(s_amp).withAimingSetPoints(ampArmSetpoints.HOME).build());
        }

        public AmpCommand createAmpScoreCommand() {
            return new AmpCommand(new AmpCommand.AmpConfiguration(s_amp).withAimingSetPoints(ampArmSetpoints.TRAP).build());
        }
        
        public AmpCommand createAmpIndexInCommand() {
            return new AmpCommand(new AmpCommand.AmpConfiguration(s_amp).withIndexState(ampIndexState.INTAKE).build());
        }
        
        public AmpCommand createAmpIndexOutCommand() {
            return new AmpCommand(new AmpCommand.AmpConfiguration(s_amp).withIndexState(ampIndexState.OUT).build());
        }

        public AmpCommand createAmpIndexStopCommand() {
            return new AmpCommand(new AmpCommand.AmpConfiguration(s_amp).withIndexState(ampIndexState.STOP).build());
        }
    }

}
