package frc.robot.subsystems;
import frc.robot.Constants.ShooterSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final SparkMax shooter = new SparkMax(ShooterSubsystem.ShooterID, MotorType.kBrushless);
    private final SparkMaxConfig shooterConfig;
    
    private double targetRpm = 0.0;

    public Shooter() {
        shooterConfig = new SparkMaxConfig();
        
        shooterConfig.idleMode(IdleMode.kCoast)
            .inverted(ShooterSubsystem.ShooterInverted)
            .smartCurrentLimit(20, 40);
        
        // Configuração de encoder
        shooterConfig.encoder.velocityConversionFactor(ShooterSubsystem.ShooterConversionFactor);
        
        // Ramp rate para proteger transmissão
        shooterConfig.openLoopRampRate(ShooterSubsystem.closedLoopRampRate);
        
        // Configuração de malha fechada interna
        shooterConfig.closedLoop
            .pid(ShooterSubsystem.kP, ShooterSubsystem.kI, ShooterSubsystem.kD, ClosedLoopSlot.kSlot0);
        
        shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Define a velocidade alvo do shooter usando malha fechada interna do SPARK MAX.
     * O feedforward pode ser configurado no PID config se necessário.
     * @param rpm Velocidade alvo em RPM
     */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        shooter.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Retorna a velocidade atual do shooter em RPM.
     */
    public double getVelocity() {
        return shooter.getEncoder().getVelocity();
    }

    /**
     * Verifica se o shooter está no setpoint com tolerância configurada.
     */
    public boolean isAtSetpoint() {
        return Math.abs(getVelocity() - targetRpm) <= ShooterSubsystem.velocityTolerance;
    }

    /**
     * Para o motor do shooter.
     */
    public void stopMotor() {
        targetRpm = 0.0;
        shooter.stopMotor();
    }

    @Override
    public void periodic() {
        double currentRpm = getVelocity();
        SmartDashboard.putNumber("ShooterSpeed", currentRpm);
        SmartDashboard.putNumber("ShooterTargetRpm", targetRpm);
        SmartDashboard.putBoolean("ShooterAtSetpoint", isAtSetpoint());
        SmartDashboard.putBoolean("ShooterReady", isAtSetpoint() && targetRpm > 0);
    }
}
