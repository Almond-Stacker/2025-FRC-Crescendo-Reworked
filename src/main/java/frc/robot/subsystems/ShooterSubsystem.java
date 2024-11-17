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
import frc.robot.Constants;
import frc.robot.States.aimingSetPoints;
import frc.robot.States.feederState;
import frc.robot.States.shooterState;

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
        armEncoder = new DutyCycleEncoder(Constants.ShooterSubsystemInfo.feederEncoder);
        armPidController = new PIDController(0.0125, 0, 0);
        armFeedforward = new ArmFeedforward(0, 0.5, 0);

        setShootingState(shooterState.STOP);
        setFeedState(feederState.STOP, 0);
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
        switch (state) {
            case SHOOT:
                setShootMotorSpeed(0.75, 0.75);
                break;
            case INTAKE:
                setShootMotorSpeed(-0.2, 0);
                break;
            case STOP:
                setShootMotorSpeed(0,0);
                break; 
        }   
        SmartDashboard.putString("Shooting Motors State", state.toString());
    }

    public void setFeedState(feederState state, double speed) {
        switch(state) {
            case OUT:
                setFeedMotorSpeed(speed);
                break;
            case INTAKE:
                setFeedMotorSpeed(-speed);
                break;
            case STOP:
                setFeedMotorSpeed(0);
                break;
        }
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

    private void setShootMotorSpeed(double topSpeed, double bottomSpeed) {
        m_bottomShootingMotor.set(bottomSpeed);
        m_topShootingMotor.set(topSpeed);
    }

    private void initializeMotors() {
        m_rightArmMotor = new CANSparkFlex(Constants.ShooterSubsystemInfo.rightArmMotorID, MotorType.kBrushless);
        m_leftArmMotor = new CANSparkFlex(Constants.ShooterSubsystemInfo.leftArmMotorID, MotorType.kBrushless);
        m_topShootingMotor = new CANSparkFlex(Constants.ShooterSubsystemInfo.topShootingMotorID, MotorType.kBrushless);
        m_bottomShootingMotor = new CANSparkFlex(Constants.ShooterSubsystemInfo.bottomShootingMotorID, MotorType.kBrushless);
        m_leftFeedMotor = new PWMSparkMax(Constants.ShooterSubsystemInfo.leftFeedingMotorID);
        m_rightFeedMotor = new PWMSparkMax(Constants.ShooterSubsystemInfo.rightFeedingMotorID);
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
