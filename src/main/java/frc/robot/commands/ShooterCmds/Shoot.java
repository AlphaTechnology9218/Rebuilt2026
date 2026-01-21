package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command{
    private final Shooter shooter;
    private final double targetRpm;

    //Utiliza o rpm para atirar o fuel até o alvo
    public Shoot(Shooter shooter, double targetRpm)
    {
        this.shooter = shooter;
        this.targetRpm = targetRpm;
        addRequirements(shooter);
    }

    public Shoot(Shooter shooter)
    {
        this(shooter, Constants.ShooterSubsystem.targetRpm);
    }

    @Override
    public void initialize()
    {
        shooter.setTargetRpmWithFF(targetRpm);
    }

    @Override
    public void execute()
    {
        
    }

    @Override
    public void end(boolean interrupted)
    {
        shooter.StopMotor();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
