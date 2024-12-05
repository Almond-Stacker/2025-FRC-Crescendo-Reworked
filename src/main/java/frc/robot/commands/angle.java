package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class angle extends Command {
    private Swerve x;
    private double rotation;

    public angle(Swerve nice, double rotatiotn) {
        this.x = nice;
        this.rotation = rotatiotn;
    }
    
    @Override
    public void execute() {
        x.drive(new Translation2d(), rotation, false, true);
    }
}
