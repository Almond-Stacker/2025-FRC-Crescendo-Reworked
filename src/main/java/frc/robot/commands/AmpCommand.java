package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.States.AmpEnums.ampArmSetpoints;
import frc.robot.States.AmpEnums.ampIndexState;
import frc.robot.States.shooterEnums.aimingSetPoints;
import frc.robot.subsystems.AmpSubsystem;

public class AmpCommand extends Command {
    private final AmpSubsystem ampSubsystem;
    private final ampArmSetpoints aState;
    private final ampIndexState iState;

    public AmpCommand(AmpConfiguration configuration) {
        this.ampSubsystem = configuration.ampSubsystem;
        this.aState = configuration.aState;
        this.iState = configuration.iState;
    }

    @Override
    public void initialize() {
        if(aState != null) {
            ampSubsystem.setArmState(aState);
        }
        if(iState != null) {
            ampSubsystem.setIndexWheelState(iState);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public static class AmpConfiguration {
        private final AmpSubsystem ampSubsystem;
        private ampArmSetpoints aState;
        private ampIndexState iState;

        public AmpConfiguration(AmpSubsystem ampSubsystem) {
            this.ampSubsystem = ampSubsystem;
        }

        public AmpConfiguration withAminingSetPoints(ampArmSetpoints state) {
            this.aState = state;
            return this;
        }

        public AmpConfiguration withIndexState(ampIndexState state) {
            this.iState = state;
            return this;
        }
    }
}
