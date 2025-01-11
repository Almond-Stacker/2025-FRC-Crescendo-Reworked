package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtil;
import frc.lib.util.SparkFlexUtil.Usage;

import frc.robot.Constants;
import frc.robot.States.intakeState;

public class IntakeSubsystem extends SubsystemBase{
    private SparkFlex m_wheelMotor;
    private SparkFlex m_floorMotor;
    private PWMSparkMax m_frontMotor;
    public IntakeSubsystem() {
        initalizeMotors();
        configureMotors();
        setIntakeState(intakeState.STOP);
        m_frontMotor.disable();
    }

    public void setIntakeState(intakeState state) {
        switch (state) {
            case INTAKE:
                setIntakeSpeed(-0.55, -0.75, 0.75);
                break;
            case OUT:
                setIntakeSpeed(0.35, -0.5, 0.5);
                break;
            case STOP:
                setIntakeSpeed(0, 0, 0);
                break;
            case AMP_INTAKE:
                setIntakeSpeed(-0.25, 0.75, 0.75);
                break;
        }
        SmartDashboard.putString("Intake State", state.toString());
    }

    private void setIntakeSpeed(double wheelValue, double frontValue, double floorValue) {
        m_wheelMotor.set(wheelValue);
        m_floorMotor.set(floorValue);
        m_frontMotor.set(frontValue);
    }

    private void initalizeMotors() {
        m_wheelMotor = new SparkFlex(Constants.IntakeSubsystemInfo.WHEEL_MOTOR_ID, MotorType.kBrushless);
        m_floorMotor = new SparkFlex(Constants.IntakeSubsystemInfo.FLOOR_MOTOR_ID, MotorType.kBrushless);
        m_frontMotor = new PWMSparkMax(Constants.IntakeSubsystemInfo.FRONT_MOTOR_ID);
    }

    private void configureMotors() {
        SparkFlexUtil.setSparkFlexBusUsage(m_wheelMotor, Usage.kVelocityOnly, IdleMode.kCoast, false, false);
        SparkFlexUtil.setSparkFlexBusUsage(m_floorMotor, Usage.kVelocityOnly, IdleMode.kCoast, false, false);    
        m_frontMotor.setInverted(false); //was true
    }
}
