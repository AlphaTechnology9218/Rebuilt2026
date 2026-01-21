package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeDeploy extends Command{
    private final Intake intake;
    private final double deployVolts;

    public IntakeDeploy(Intake intake, double deployVolts)
    {
        this.intake = intake;
        this.deployVolts = deployVolts;
        addRequirements(intake);
    }


    public IntakeDeploy(Intake intake)
    {
        this(intake, Constants.IntakeSubsystem.deployVoltage);
    }

    @Override
    public void initialize()
    {
        intake.setDeployVoltage(deployVolts);
    }

    @Override
    public void execute()
    {
        intake.setDeployVoltage(deployVolts);
    }

    @Override
    public void end(boolean interrupted)
    {
        intake.stopDeploy();
    }

    @Override
    public boolean isFinished()
    {
        if (deployVolts > 0)
        {
            return intake.isAtDeployLimit();
        }
        else if (deployVolts < 0)
        {
            return intake.isAtRetractLimit();
        }
        return false;
    }
}
