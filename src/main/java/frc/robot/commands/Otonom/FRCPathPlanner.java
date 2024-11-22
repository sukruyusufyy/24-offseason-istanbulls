package frc.robot.commands.Otonom;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCommand;






public class FRCPathPlanner {

    public final static SendableChooser<Command> autoChooser=AutoBuilder.buildAutoChooser();
    
    public static void SetPathPlannerSettings(){
      setDashboard();
      CommandNameEntry();
      
    }
    public static void setDashboard(){
        SmartDashboard.putData("Auto Mod", autoChooser);
        SmartDashboard.putBoolean("is AutoBuilder configure?", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean(" is pathfinding configure?", AutoBuilder.isPathfindingConfigured());
    }

    public static void CommandNameEntry(){

    NamedCommands.registerCommand("shoot", new ShootCommand(RobotContainer.getIntakeSubsystem(), RobotContainer.getShooterSubsystem()));

    // NamedCommands.registerCommand("intake", new IntakeCommand(RobotContainer.getIntakeSubsystem(), RobotContainer.getMZ80(), true).withTimeout(5));
    
    }
    
}