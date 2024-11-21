package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.AmpSubsystemInfo;
import frc.robot.States.AmpEnums.*;
import frc.robot.Constants.AmpSubsystemInfo;

public class AmpSubsystem extends SubsystemBase{
    private CANSparkMax m_rightArmMotor;
    private CANSparkMax m_leftArmMotor;
    private PWMSparkMax m_indexMotor;
    private DutyCycleEncoder armEncoder;
    
    private PIDController armPID;
    private double armPosition;
    private double armSpeed;

    public AmpSubsystem() {
        initalizeMotors();
        configureMotors();
        armEncoder = new DutyCycleEncoder(AmpSubsystemInfo.ampEncoder);
        armPID = new PIDController(0.008, 0, 0);
    }

    @Override
    public void periodic() {
        armPosition = armEncoder.getAbsolutePosition() * 360;
        armSpeed = armPID.calculate(armPosition);
        setArmSpeed(armSpeed);
        setSmartDashboardData();
    }

    public void setArmState(ampArmSetpoints state) {
        armPID.setSetpoint(state.getValue());
        SmartDashboard.putString("Amp arm state", state.toString());
        SmartDashboard.putNumber("Amp Arm target", state.getValue());
    }

    public void setIndexWheelState(ampIndexState state) {
        m_indexMotor.set(state.getValue());
        SmartDashboard.putString("Amp Index State", state.toString());
    }

    private void setArmSpeed(double speed) {
        m_rightArmMotor.set(speed);
        m_leftArmMotor.set(speed);
    }

    private void initalizeMotors() {
        m_rightArmMotor = new CANSparkMax(AmpSubsystemInfo.RIGHT_ARM_MOTOR_ID, MotorType.kBrushless);
        m_leftArmMotor = new CANSparkMax(AmpSubsystemInfo.LEFT_ARM_MOTOR_ID, MotorType.kBrushless);
        m_indexMotor = new PWMSparkMax(AmpSubsystemInfo.AMP_SCORER_MOTOR_ID);
        armEncoder = new DutyCycleEncoder(AmpSubsystemInfo.AMP_ENCODER);
    }

    private void configureMotors() {
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_leftArmMotor, Usage.kVelocityOnly);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_rightArmMotor, Usage.kVelocityOnly);

        m_rightArmMotor.setIdleMode(IdleMode.kCoast);
        m_leftArmMotor.setIdleMode(IdleMode.kCoast);

        m_rightArmMotor.setInverted(false);
        m_leftArmMotor.setInverted(true);
    }

    private void setSmartDashboardData() {
        SmartDashboard.putNumber("Amp Arm position", armPosition);
        SmartDashboard.putNumber("Amp Arm Motor output", armSpeed);
    }
}
