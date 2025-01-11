package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtil;
import frc.lib.util.SparkFlexUtil.Usage;
import frc.robot.Constants.ShooterSubsystemInfo;
import frc.robot.States.shooterEnums.*;

public class ShooterSubsystem extends SubsystemBase{
    // control arm to aim shooter
    private SparkFlex m_rightArmMotor;
    private SparkFlex m_leftArmMotor;
    private DutyCycleEncoder armEncoder;
    private PIDController armPidController;
    private double armSpeed;
    private double armPosition;
    private boolean inRange;
    
    // shoot note out
    private SparkFlex m_topShootingMotor;
    private SparkFlex m_bottomShootingMotor;

    // feeds into shooter
    private PWMSparkMax m_leftFeedMotor;
    private PWMSparkMax m_rightFeedMotor;

    public ShooterSubsystem() {
        initializeMotors();
        configureMotors();
        armEncoder = new DutyCycleEncoder(ShooterSubsystemInfo.FEEDER_ENCODER);
        armPidController = new PIDController(0.0125, 0, 0);
        
        setShootingState(shooterState.STOP);
        setFeedState(feederState.STOP);
        setArmState(aimingSetPoints.HOME);
    }

    @Override
    public void periodic() {
        inRange = false;
        armPosition = armEncoder.get() * 360;
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
        m_rightArmMotor = new SparkFlex(ShooterSubsystemInfo.RIGHT_ARM_MOTOR_ID, MotorType.kBrushless);
        m_leftArmMotor = new SparkFlex(ShooterSubsystemInfo.LEFT_ARM_MOTOR_ID, MotorType.kBrushless);
        m_topShootingMotor = new SparkFlex(ShooterSubsystemInfo.TOP_SHOOTING_MOTOR_ID, MotorType.kBrushless);
        m_bottomShootingMotor = new SparkFlex(ShooterSubsystemInfo.BOTTOM_SHOOTING_MOTOR_ID, MotorType.kBrushless);
        m_leftFeedMotor = new PWMSparkMax(ShooterSubsystemInfo.LEFT_FEEDING_MOTOR_ID);
        m_rightFeedMotor = new PWMSparkMax(ShooterSubsystemInfo.RIGHT_FEEDING_MOTOR_ID);
    }

    private void configureMotors() {
        SparkFlexUtil.setSparkFlexBusUsage(m_rightArmMotor, Usage.kVelocityOnly, IdleMode.kBrake, false, true);
        SparkFlexUtil.setSparkFlexBusUsage(m_leftArmMotor, Usage.kVelocityOnly, IdleMode.kBrake, false, false);
        SparkFlexUtil.setSparkFlexBusUsage(m_topShootingMotor, Usage.kVelocityOnly, IdleMode.kCoast, false, false);
        SparkFlexUtil.setSparkFlexBusUsage(m_bottomShootingMotor, Usage.kVelocityOnly, IdleMode.kCoast, false, false);

        m_leftFeedMotor.setInverted(false);
        m_rightFeedMotor.setInverted(true);
    }

    private void putSmartDashboardData() {
        SmartDashboard.putBoolean("Shooter arm in range", inRange);
        SmartDashboard.putNumber("Encoder Value", armPosition);
        SmartDashboard.putNumber("Arm Motor Output", armSpeed);
        SmartDashboard.putData(armEncoder);
    }
}