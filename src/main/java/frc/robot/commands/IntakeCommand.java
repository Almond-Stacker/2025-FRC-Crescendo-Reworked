package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.States.intakeState;

public class IntakeCommand extends Command {
    private IntakeSubsystem intake;
    private intakeState state;

    public IntakeCommand(IntakeSubsystem intake, intakeState state) {
        this.intake = intake;
        this.state = state;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}