package frc.robot.commands.TestCmds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterSpinTest extends Command {
  public static final String kTargetRpmKey = "ShooterTest/TargetRPM";

  private final Shooter shooter;
  private final double defaultRpm;

  public ShooterSpinTest(Shooter shooter, double defaultRpm) {
    this.shooter = shooter;
    this.defaultRpm = defaultRpm;
    addRequirements(shooter);
  }

  public ShooterSpinTest(Shooter shooter) {
    this(shooter, Constants.ShooterSubsystem.targetRpm);
  }

  @Override
  public void initialize() {
    if (!SmartDashboard.containsKey(kTargetRpmKey)) {
      SmartDashboard.putNumber(kTargetRpmKey, defaultRpm);
    }
  }

  @Override
  public void execute() {
    double rpm = SmartDashboard.getNumber(kTargetRpmKey, defaultRpm);
    if (rpm < 0) rpm = 0;
    shooter.setTargetRpm(rpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}



