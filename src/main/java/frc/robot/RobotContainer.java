// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.cscore.UsbCamera;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import org.opencv.core.Mat;
import org.opencv.core.Point;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/maxSwerve"));
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();
  public static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final PnumaticSubsystem m_climb = new PnumaticSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverXbox = new CommandPS5Controller(0);
  final CommandPS5Controller testPS5 = new CommandPS5Controller(1);
  private RobotContainer m_robotContainer;

  public Command driveAdjustToObject;
  private final ShootCommand m_ShootCommand = new ShootCommand(m_intake, m_shooter);
  CvSink cvSink;
  UsbCamera usbCamera;

  private void configureAutoCommmands() {
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("shoot", m_shooter.runShooterCommand());
    NamedCommands.registerCommand("shootstop", m_shooter.runShooterCommand());
    configureAutoCommmands();
    // Configure the trigger bindings
    configureBindings();

    CvSource outputStream = CameraServer.putVideo("Rectangle", 100, 100);
    Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);

    driveAdjustToObject = drivebase.driveCommand(
          () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
          () -> m_vision.getObjectYaw());
  }

  private void configureBindings() {
    m_chooser.setDefaultOption("bluecapraz", drivebase.getAutonomousCommand("bluecapraz"));
    m_chooser.addOption("Mehmet", drivebase.getAutonomousCommand("Mehmet"));

    SmartDashboard.putData(m_chooser);

    driverXbox.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());

    driverXbox.povUp().onTrue(m_arm.armUpperReset());
    driverXbox.povDown().onTrue(m_arm.armLowerReset());

    // driverXbox.button(10).whileTrue(m_intake.stopFeederCommand());

    // Üçgene basıldığında otomatik kol ayarlamayı aç
    driverXbox.triangle().toggleOnTrue(
        m_arm.setArmAngleCommand(() -> {
          m_arm.isPIDActive = true;
          SmartDashboard.putBoolean("Is auto arm on", true);
          return m_arm.calculateArmAngle(m_vision.getDistanceToSpeaker());
        }));

    // Modun kapandığını SmartDashboarda haber ver
    driverXbox.triangle().toggleOnFalse(
        new InstantCommand(() -> {
          m_arm.isPIDActive = false;
          SmartDashboard.putBoolean("Is auto arm on", false);
        }));

    // driverXbox.circle().toggleOnTrue(
    //     new InstantCommand(() -> {
    //       m_intake.runIntakeOut();
    //       m_intake.runFeederOut();
    //       SmartDashboard.putBoolean("Is feeder out", false);
    //     }));

    // driverXbox.circle().toggleOnFalse(
    //     new InstantCommand(() -> {
    //       m_intake.stopIntake();
    //       m_intake.stopFeeder();
    //       SmartDashboard.putBoolean("Is feeder out stop ", false);
    //     }));

    driverXbox.button(12).whileTrue(
      new ConditionalCommand(
        driveAdjustToObject,
        new InstantCommand(),
        () -> { return m_vision.isSeeingObject(); }
      )
    );

    driverXbox.povLeft().onTrue(
        new InstantCommand(() -> {
          m_climb.ClimbOn();
          System.out.println("Is pnumutic open ");
        }));

    driverXbox.povRight().onTrue(
        new InstantCommand(() -> {
          m_climb.climof();
          System.out.println("Is pnumutic off ");
        }));

    driverXbox.circle().toggleOnTrue(
        new InstantCommand(() -> {
          SmartDashboard.putBoolean("Target Speaker ", true);
        }));
    driverXbox.circle().toggleOnFalse(
        new InstantCommand(() -> {
          SmartDashboard.putBoolean("Target Speaker ", false);
        }));

    driverXbox.L1().whileTrue(
        new StartEndCommand(() -> m_arm.setVoltage(4), () -> m_arm.setVoltage(ArmConstants.kArmStatic), m_arm));
    driverXbox.L2().whileTrue(
        new StartEndCommand(() -> m_arm.setVoltage(-4), () -> m_arm.setVoltage(ArmConstants.kArmStatic), m_arm));

    driverXbox.R1().whileTrue(new ShootCommand(m_intake, m_shooter));
    driverXbox.R2().whileTrue(new StartEndCommand(
        () -> {
          m_intake.runIntakeIn();
        },
        () -> {
          m_intake.stopIntake();
        }));
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Arm Current Angle", m_arm.getAngle());
    SmartDashboard.putNumber("Arm Angle Setpoint", m_arm.m_armAngleSetpoint);
    SmartDashboard.putNumber("Arm Written Voltage", m_arm.m_lastVoltage);

    SmartDashboard.putBoolean("is arm at upper limit", !m_arm.m_armUpperLimit.get());
    SmartDashboard.putBoolean("is arm at lower limit", !m_arm.m_armLowerLimit.get());
    SmartDashboard.putBoolean("has object", !m_intake.getSensorReading());

    SmartDashboard.putBoolean("is seeing object", m_vision.isSeeingObject());
    SmartDashboard.putNumber("Best Object Yaw", m_vision.getObjectYaw());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected(); // path takip kodu */
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public static IntakeSubsystem getIntakeSubsystem() {
    return m_intake;
  }

  public static ShooterSubsystem getShooterSubsystem() {
    return m_shooter;
  }

}