package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.CANSparkFlexUtil;
import frc.lib.util.CANSparkFlexUtil.Usage;
import frc.robot.Constants.ShooterSubsystemInfo;
import frc.robot.States.shooterEnums.*;

public class ShooterSubsystem extends SubsystemBase{
    // control arm to aim shooter
    private CANSparkFlex m_rightArmMotor;
    private CANSparkFlex m_leftArmMotor;
    private DutyCycleEncoder armEncoder;
    private PIDController armPidController;
    private ArmFeedforward armFeedforward;
    private double armSpeed;
    private double armPosition;
    private boolean inRange;
    
    // shoot note out
    private CANSparkFlex m_topShootingMotor;
    private CANSparkFlex m_bottomShootingMotor;

    // feeds into shooter
    private PWMSparkMax m_leftFeedMotor;
    private PWMSparkMax m_rightFeedMotor;

    public ShooterSubsystem() {
        initializeMotors();
        configureMotors();
        armEncoder = new DutyCycleEncoder(ShooterSubsystemInfo.FEEDER_ENCODER);
        armPidController = new PIDController(0.0125, 0, 0);
        armFeedforward = new ArmFeedforward(0, 0.5, 0);

        setShootingState(shooterState.STOP);
        setFeedState(feederState.STOP);
        setArmState(aimingSetPoints.HOME);
    }

    @Override
    public void periodic() {
        inRange = false;
        armPosition = armEncoder.getAbsolutePosition() * 360;
        armSpeed = armPidController.calculate(armPosition);

        if(armPosition > aimingSetPoints.MIN.getValue() && armPosition < aimingSetPoints.MAX.getValue()) {
            setAimMotorSpeed(armSpeed);
            inRange = true;
        }
        putSmartDashboardData();
    }

    public void setShootingState(shooterState state) {
        setShootMotorSpeed(state.getValue());
        SmartDashboard.putString("Shooting Motors State", state.toString());
    }

    public void setFeedState(feederState state) {
        setFeedMotorSpeed(state.getValue());
        SmartDashboard.putString("Feeder State", state.toString());
    }

    public void setArmState(aimingSetPoints state) {
        armPidController.setSetpoint(state.getValue());
        // TODO: add feedforward later 
        SmartDashboard.putString("Shooter Arm State", state.toString());
        SmartDashboard.putNumber("Shooter arm setpoint", state.getValue());
    }
    
    private void setFeedMotorSpeed(double speed) {
        m_rightFeedMotor.set(speed);
        m_leftFeedMotor.set(speed);
    }

    private void setAimMotorSpeed(double speed) {
        m_rightArmMotor.set(speed);
        m_leftArmMotor.set(speed);
    }

    private void setShootMotorSpeed(double Speed) {
        m_bottomShootingMotor.set(Speed);
        m_topShootingMotor.set(Speed);
    }

    private void initializeMotors() {
        m_rightArmMotor = new CANSparkFlex(ShooterSubsystemInfo.RIGHT_ARM_MOTOR_ID, MotorType.kBrushless);
        m_leftArmMotor = new CANSparkFlex(ShooterSubsystemInfo.LEFT_ARM_MOTOR_ID, MotorType.kBrushless);
        m_topShootingMotor = new CANSparkFlex(ShooterSubsystemInfo.TOP_SHOOTING_MOTOR_ID, MotorType.kBrushless);
        m_bottomShootingMotor = new CANSparkFlex(ShooterSubsystemInfo.BOTTOM_SHOOTING_MOTOR_ID, MotorType.kBrushless);
        m_leftFeedMotor = new PWMSparkMax(ShooterSubsystemInfo.LEFT_FEEDING_MOTOR_ID);
        m_rightFeedMotor = new PWMSparkMax(ShooterSubsystemInfo.RIGHT_FEEDING_MOTOR_ID);
    }

    private void configureMotors() {
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_rightArmMotor, Usage.kVelocityOnly);
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_leftArmMotor, Usage.kVelocityOnly);
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_topShootingMotor, Usage.kVelocityOnly);
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_bottomShootingMotor, Usage.kVelocityOnly);

        m_rightArmMotor.setIdleMode(IdleMode.kBrake);
        m_leftArmMotor.setIdleMode(IdleMode.kBrake);
        m_rightArmMotor.setInverted(true);
        m_leftArmMotor.setInverted(false);

        m_bottomShootingMotor.setIdleMode(IdleMode.kCoast);
        m_topShootingMotor.setIdleMode(IdleMode.kCoast);
        m_bottomShootingMotor.setInverted(false);
        m_topShootingMotor.setInverted(false);

        m_leftFeedMotor.setInverted(true);
        m_rightFeedMotor.setInverted(true);
    }

    private void putSmartDashboardData() {
        SmartDashboard.putBoolean("Shooter arm in range", inRange);
        SmartDashboard.putNumber("Encoder Value", armPosition);
        SmartDashboard.putNumber("Arm Motor Output", armSpeed);
        SmartDashboard.putData(armEncoder);
    }
}