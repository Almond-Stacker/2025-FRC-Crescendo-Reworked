package frc.robot;

public class States {
    public enum intakeState {
        INTAKE, 
        OUT,
        STOP,
        AMP_INTAKE
    }

    public enum aimingSetPoints {
        MIN(56),
        MAX(169.7),
        HOME(60),
        FLOAT(70),
        INTAKE(MIN.getValue() + 20),
        TRAP(MIN.getValue()),
        FAR(MIN.getValue()),
        NEAR(MIN.getValue()),
        CLIMB(MIN.getValue());
    

        private final double value;

        aimingSetPoints(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    public enum ampState {
        INTAKE,
        OUT,
        STOP
    }

    public enum feederState {
        INTAKE,
        OUT,
        STOP
    }

    public enum shooterState {
        INTAKE,
        SHOOT,
        STOP
    }

    // public enum shooterAimState {
    //     HOME,
    //     INTAKE,
    //     TRAP,
    //     FAR,
    //     NEAR,
    //     CLIMB,
    //     FLOAT
    // }

    public enum ampArmState {
        HOME,
        TRAP,
        AMP
    }
}
