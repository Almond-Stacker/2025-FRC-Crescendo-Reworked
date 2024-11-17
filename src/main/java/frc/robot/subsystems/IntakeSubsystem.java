package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.CANSparkFlexUtil;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkFlexUtil.Usage;

import frc.robot.Constants;
import frc.robot.States.intakeState;

public class IntakeSubsystem extends SubsystemBase{
    private CANSparkFlex m_wheelMotor;
    private CANSparkFlex m_floorMotor;
    private PWMSparkMax m_frontMotor;
    public IntakeSubsystem() {
        initalizeMotors();
        configureMotors();
        setIntakeState(intakeState.STOP);
    }

    public void setIntakeState(intakeState state) {
        switch (state) {
            case INTAKE:
                setIntakeSpeed(-0.55, -0.75, 0.75);
                break;
            case OUT:
                setIntakeSpeed(0.55, -0.5, 0.5);
                break;
            case STOP:
                setIntakeSpeed(0, 0, 0);
                break;
            case AMP_INTAKE:
                setIntakeSpeed(0.25, 0.75, 0.75);
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
        m_wheelMotor = new CANSparkFlex(Constants.IntakeSubsystemInfo.wheelMotorID, MotorType.kBrushless);
        m_floorMotor = new CANSparkFlex(Constants.IntakeSubsystemInfo.floorMotorID, MotorType.kBrushless);
        m_frontMotor = new PWMSparkMax(Constants.IntakeSubsystemInfo.frontMotorID);
    }

    private void configureMotors() {
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_wheelMotor, Usage.kVelocityOnly);
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_floorMotor, Usage.kVelocityOnly);

        m_wheelMotor.setInverted(false);
        m_frontMotor.setInverted(false); //was true
        m_floorMotor.setInverted(false); //was true
    }
}
