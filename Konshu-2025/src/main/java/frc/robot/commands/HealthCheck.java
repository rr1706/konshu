package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.thethriftybot.ThriftyNova.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FunnelMotor;

public class HealthCheck extends SequentialCommandGroup{
    private Arm arm;
    private AlgaeArm algaearm;
    private AlgaeIntake algaeintake;
    private Climber climber;
    private CommandSwerveDrivetrain drive;
    private CoralArm coralarm;
    private 
}


