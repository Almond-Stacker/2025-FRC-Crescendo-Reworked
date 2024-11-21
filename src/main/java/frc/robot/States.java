package frc.robot;

public class States {
    public enum intakeState {
        INTAKE, 
        OUT,
        STOP,
        AMP_INTAKE,
        AMP_OUT
    }

    public class shooterEnums {
        public enum aimingSetPoints {
            MIN(56),
            MAX(169.7),
            HOME(60),
            FLOAT(70),
            INTAKE(),
            TRAP(),
            FAR(),
            NEAR(),
            CLIMB();
        

            private final double value;

            aimingSetPoints(double value) {
                this.value = value;
            }

            aimingSetPoints() {
                this.value = 60;
            }

            public double getValue() {
                return value;
            }
        }

        public enum feederState {
            INTAKE(0.2),
            OUT(-0.2),
            STOP(0);

            private final double value;

            feederState(double value) {
                this.value = value;
            }

            public double getValue() {
                return value;
            }
        }

        public enum shooterState {
            SHOOT(0.7),
            FEEDBACK(-0.2),
            STOP(0);

            private final double value;

            shooterState(double value) {
                this.value = value;
            }

            public double getValue() {
                return value;
            }
        }
    }
    
    public class AmpEnums {
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
            HOME(306),
            MIN(304),
            TRAP(),
            AIM(),
            MAX(172),
            UP(),
            DOWN();

            private double value;

            ampArmSetpoints(double value) {
                this.value = value;
            }

            ampArmSetpoints() {
                this.value = 306;
            }

            public double getValue() {
                return value;
            }
            
            public void setValue(double value) {
                this.value = value;
            }
        }
    }
}
