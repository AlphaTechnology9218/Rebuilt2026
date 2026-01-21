package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCollector extends Command {
    private final Intake intake;
    private final double targetRpm;

    //rodas utilizando pid e feedfoward para coletar a bola
    public IntakeCollector(Intake intake, double targetRpm)
    {
        this.intake = intake;
        this.targetRpm = targetRpm;
        addRequirements(intake);
    }


    public IntakeCollector(Intake intake)
    {
        this(intake, Constants.IntakeSubsystem.collectorTargetRpm);
    }

    @Override
    public void initialize()
    {
        intake.setCollectorTargetRpmWithFF(targetRpm);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public void end(boolean interrupted)
    {
        intake.stopCollector();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
