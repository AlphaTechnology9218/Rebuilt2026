package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//LINEAR 1/27
//INTAKE 1/4
public class Intake extends SubsystemBase {
    Double LinearSetpoint = 0.0;
    SparkMax IntakeMotor = new SparkMax(IntakeSubsystem.IntakeMotorID, MotorType.kBrushless);
    SparkMax LinearMotor = new SparkMax(IntakeSubsystem.LinearMotorID, MotorType.kBrushless);
    SparkMaxConfig intakeConfig;
    SparkMaxConfig LinearConfig;

    private boolean isDeployed = false;

    private final PIDController collectorPid = new PIDController(
        IntakeSubsystem.collectorKp,
        IntakeSubsystem.collectorKi,
        IntakeSubsystem.collectorKd
    );
    private final SimpleMotorFeedforward collectorFF = new SimpleMotorFeedforward(
        IntakeSubsystem.collectorKsVolts,
        IntakeSubsystem.collectorKvVoltPerRpm,
        IntakeSubsystem.collectorKaVoltPerRpmPerSec
    );
    private double collectorTargetRpm = 0.0;
    private boolean collectorClosedLoopEnabled = false;

    public Intake()
    {
        intakeConfig = new SparkMaxConfig();
        LinearConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20,40).inverted(IntakeSubsystem.intakeInverted);

        intakeConfig.encoder.velocityConversionFactor(IntakeSubsystem.intakeConversionFactor);

        intakeConfig.closedLoop.pid(IntakeSubsystem.iP,IntakeSubsystem.iI,IntakeSubsystem.iD, ClosedLoopSlot.kSlot0);

        LinearConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40,60).inverted(IntakeSubsystem.LinearInverted);
        
        LinearConfig.encoder.positionConversionFactor(IntakeSubsystem.LinearConversionFactor).velocityConversionFactor(IntakeSubsystem.LinearConversionFactor);

        LinearConfig.closedLoop.pid(IntakeSubsystem.lP, IntakeSubsystem.lD, IntakeSubsystem.lI, ClosedLoopSlot.kSlot0);

        IntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        LinearMotor.configure(LinearConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
    }

    public void LinearToCollect()
    {
        LinearMotor.getClosedLoopController().setSetpoint(LinearSetpoint, ControlType.kPosition);
    }

    public void LinearToInitial()
    {
        LinearMotor.getClosedLoopController().setSetpoint(0.0, ControlType.kPosition);
    }

    public void LinearStop()
    {
        LinearMotor.stopMotor();
    }

    public boolean LinearAtsetpoint()
    {
        return LinearMotor.getClosedLoopController().isAtSetpoint();
    }

    public void IntakeActive(double velocitySetpoint)
    {
        IntakeMotor.getClosedLoopController().setSetpoint(velocitySetpoint, ControlType.kVelocity);
    }

    public double getLinearPosition()
    {
        return LinearMotor.getEncoder().getPosition();
    }

    public boolean isDeployed()
    {
        return isDeployed;
    }

    public void stopDeploy()
    {
        LinearMotor.stopMotor();
    }

    public void retractUntilZero()
    {
        isDeployed = false;
        setDeployVoltage(-IntakeSubsystem.deployVoltage);
    }

    public void deployUntilMax()
    {
        isDeployed = true;
        setDeployVoltage(IntakeSubsystem.deployVoltage);
    }

    public void toggleDeploy()
    {
        isDeployed = !isDeployed;
    }

    public void applyDeploy()
    {
        if (isDeployed)
        {
            deployUntilMax();
        } else
        {
            retractUntilZero();
        }
    }

    public void setDeployVoltage(double volts)
    {
        double pos = getLinearPosition();

        if (volts > 0 && pos >= IntakeSubsystem.deployMaxPosition)
        {
            stopDeploy();
            return;
        }

        if (volts < 0 && pos <= 0.0)
        {
            stopDeploy();
            return;
        }

        LinearMotor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    public boolean isAtDeployLimit()
    {
        double pos = getLinearPosition();
        return pos >= IntakeSubsystem.deployMaxPosition - IntakeSubsystem.deployHoldEpsilon;
    }

    public boolean isAtRetractLimit()
    {
        double pos = getLinearPosition();
        return pos <= IntakeSubsystem.deployHoldEpsilon;
    }



    public void setCollectorTargetRpmWithFF(double rpm)
    {
        collectorTargetRpm = rpm;
        collectorClosedLoopEnabled = true;
        collectorPid.reset();
    }

    public double getCollectorVelocity()
    {
        return IntakeMotor.getEncoder().getVelocity();
    }

    public void stopCollector()
    {
        collectorClosedLoopEnabled = false;
        collectorTargetRpm = 0.0;
        IntakeMotor.stopMotor();
    }

    public void stopintake()
    {
        IntakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double collectorRpm = getCollectorVelocity();
        SmartDashboard.putNumber("IntakeSpeed", collectorRpm);
        SmartDashboard.putNumber("CollectorTargetRpm", collectorTargetRpm);
        SmartDashboard.putBoolean("CollectorClosedLoopEnabled", collectorClosedLoopEnabled);
        SmartDashboard.putNumber("LinearPos", getLinearPosition());
        SmartDashboard.putBoolean("DeployIsDeployed", isDeployed);

        if (collectorClosedLoopEnabled)
        {
            double pidOut = collectorPid.calculate(collectorRpm, collectorTargetRpm);
            double ffVolts = collectorFF.calculate(collectorTargetRpm);
            double volts = MathUtil.clamp(ffVolts + pidOut, -12.0, 12.0);
            IntakeMotor.setVoltage(volts);
            SmartDashboard.putNumber("CollectorAppliedVolts", volts);
        }
    }
}
