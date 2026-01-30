// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.IntakeCoralINCmd;
import frc.robot.commands.IntakeCoraloutCmd;
// import frc.robot.subsystems.Intake;
// import frc.robot.commands.ShooterCmds.Shoot;
// import frc.robot.commands.IntakeCmds.IntakeCollector;
// import frc.robot.commands.IntakeCmds.IntakeDeploy;
// import frc.robot.commands.IntakeCmds.IntakeDeployToggle;
import frc.robot.commands.TestCmds.ShooterSpinTest;
import frc.robot.subsystems.CoralIntake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                          "swerve"));
  //private final Shooter shooter = new Shooter();
  // private final Intake intake = new Intake();
  private final CoralIntake CIntake = new CoralIntake();
  
  private final static CommandPS4Controller m_driverController = new CommandPS4Controller(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_systemController = new CommandXboxController(Constants.OperatorConstants.kSystemControllerPort);
  private SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    

    //Qualquer comando nomeado que será utilizado no pathplannner deve ser registrado
    //Exemplo:
    //NamedCommands.registerCommand("IntakeInCmd", new IntakeInCmd(intake));
    
    // Configure the trigger bindings
    configureBindings();
      m_systemController.rightTrigger().whileTrue(new IntakeCoraloutCmd(CIntake));
      m_systemController.leftTrigger().whileTrue(new IntakeCoralINCmd(CIntake));

    initializeChooser();
    



      //Aplica as DeadBands e Inverte os controles por que os joysticks
      //estão positivos para trás e para direita
      //enquanto o movimento do robô está positivo
      //para frente e para esquerda
      //joystick esquerdo controla movimentação 
      //joystick direito controla a velocidade angular do robô
      //R2 (gatilho direito) controla escala de velocidade
      Command baseDriveCommand = drivebase.driveCommand(
     () -> MathUtil.applyDeadband(-m_driverController.getLeftY() * Math.max(-m_driverController.getR2Axis() + 1, 0.3), OperatorConstants.LEFT_X_DEADBAND),
     () -> MathUtil.applyDeadband(- m_driverController.getLeftX()* Math.max(-m_driverController.getR2Axis()+ 1, 0.3), OperatorConstants.LEFT_Y_DEADBAND),
     () -> -m_driverController.getRightX() * Math.max(m_driverController.getR2Axis()+ 1, 0.565));

    

      drivebase.setDefaultCommand(baseDriveCommand);
      }

    public static void setRightRumbleDriver(double rumble){
      m_driverController.getHID().setRumble(RumbleType.kRightRumble, rumble);
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclas   ses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.cross().onTrue(Commands.runOnce(drivebase::zeroGyro));
    m_driverController.circle().onTrue(Commands.runOnce(drivebase::resetIMU));
    
   // new Trigger(DriverStation::isTestEnabled)
        //.and(m_systemController.rightTrigger(0.1))
       // .whileTrue(new ShooterSpinTest(shooter));
  }


  
  private void initializeChooser(){

  }   

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }

  public Command getTestLockCommand() {
    // return Commands.run(drivebase::lock, drivebase);
    return Commands.none();
  }

  public void setDriveMode()
  {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    // drivebase.setMotorBrake(brake);
  }
}
