package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.intakeState;
import frc.robot.States.AmpEnums.ampArmSetpoints;
import frc.robot.States.shooterEnums.aimingSetPoints;
import frc.robot.States.shooterEnums.feederState;
import frc.robot.States.shooterEnums.shooterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private final feederState fState;
    private final aimingSetPoints aState;
    private final shooterState sState; 

    public ShooterCommand(ShooterConfiguration configuraton) {
        this.shooterSubsystem = configuraton.shooter;
        this.fState = configuraton.fState;
        this.aState = configuraton.aState;
        this.sState = configuraton.sState;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        if(aState != null) {
            shooterSubsystem.setArmState(aState);
        }
        if(fState != null) {
            shooterSubsystem.setFeedState(fState);
        }
        if(sState != null) {
            shooterSubsystem.setShootingState(sState);
        }
    }

    public static class ShooterConfiguration {
        private final ShooterSubsystem shooter;
        private feederState fState;
        private aimingSetPoints aState;
        private shooterState sState;

        public ShooterConfiguration (ShooterSubsystem shooter) {
            this.shooter = shooter;
            fState = null;
            aState = null;
            sState = null;
        }

        public ShooterConfiguration withFeederState(feederState state) { //, double speed) {
            this.fState = state;
            return this;
        }

        public ShooterConfiguration withShooterState(shooterState state) {
            this.sState = state;
            return this;
        }

        public ShooterConfiguration withAimState(aimingSetPoints state) {
            this.aState = state;
            return this;
        }

        public ShooterConfiguration build() {
            return this;
        }
    }
}
