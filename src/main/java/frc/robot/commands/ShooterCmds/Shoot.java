package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command{
       Shooter ShooterSubsystem;
       Timer timer;
       


    public Shoot(Shooter subsystem){
        this.ShooterSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
    }

    @Override
    public void execute() {
        ShooterSubsystem.intakemotion (2);
        timer.start();
        
    }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.intakestop();

    }
    }
