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

    public enum ampIndexState {
        INTAKE(1),
        OUT(-1),
        STOP(0);

        private final int value;

        ampIndexState(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum ampArmSetpoints {
        HOME(174),
        MIN(10),
        TRAP(0),
        AIM(23.1),
        MAX(180),
        UP(0),
        DOWN(0);

        private double value;

        ampArmSetpoints(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
        
        public void setValue(double value) {
            this.value = value;
        }
    }
}
