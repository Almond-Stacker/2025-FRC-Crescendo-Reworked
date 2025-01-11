package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkMaxUtil;
import frc.lib.util.SparkMaxUtil.Usage;
import frc.robot.Constants.AmpSubsystemInfo;
import frc.robot.States.AmpEnums.*;
import frc.robot.Constants.AmpSubsystemInfo;

public class AmpSubsystem extends SubsystemBase{
    private SparkMax m_rightArmMotor;
    private SparkMax m_leftArmMotor;
    private PWMSparkMax m_indexMotor;
    private DutyCycleEncoder armEncoder;
    
    private PIDController armPID;
    private double armPosition;
    private double armSpeed;

    public AmpSubsystem() {
        initalizeMotors();
        configureMotors();
        armPID = new PIDController(0.0032, 0, 0);
        setArmState(ampArmSetpoints.HOME);
    }

    @Override
    public void periodic() {
        armPosition = armEncoder.get() * 360;
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
        m_rightArmMotor = new SparkMax(AmpSubsystemInfo.RIGHT_ARM_MOTOR_ID, MotorType.kBrushless);
        m_leftArmMotor = new SparkMax(AmpSubsystemInfo.LEFT_ARM_MOTOR_ID, MotorType.kBrushless);
        m_indexMotor = new PWMSparkMax(AmpSubsystemInfo.AMP_SCORER_MOTOR_ID);
        armEncoder = new DutyCycleEncoder(AmpSubsystemInfo.AMP_ENCODER);
    }
    private void configureMotors() {
        SparkMaxUtil.setSparkMaxBusUsage(m_leftArmMotor, Usage.kAll, IdleMode.kBrake, false, true);
        SparkMaxUtil.setSparkMaxBusUsage(m_rightArmMotor, Usage.kAll, IdleMode.kBrake, false, false);
    }

    private void setSmartDashboardData() {
        SmartDashboard.putNumber("Amp Arm position", armPosition);
        SmartDashboard.putNumber("Amp Arm Motor output", armSpeed);
    }
}
