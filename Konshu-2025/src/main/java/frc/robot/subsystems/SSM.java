package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.core.CoreCANdi;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeFromFunnel;

public class SSM extends SubsystemBase {
    private final Arm m_arm;
    private final Elevator m_elevator;
    public enum States {DISABLED, L1, L2, L3, L4, LOADINGSTATION, PROCESSOR, BARGE, GROUNDALGAE, ALGAELOW, ALGAEHIGH, Climb};
    private boolean m_elevatorPauseHigh, m_elevatorPauseLow, m_armPauseHigh, m_armPauseLow;
    private States m_setpoint, m_queuedSetpoint;
    private double m_as, m_es;

    // First constructor accepts a constant as an initial setpoint and starts moving there immediately via periodic
    public SSM(Arm arm, Elevator elevator, States setpoint) {
        m_arm = arm;
        m_elevator = elevator;
        m_setpoint = States.DISABLED;
        m_queuedSetpoint = setpoint;
        m_elevatorPauseHigh = false;
        m_elevatorPauseLow = false;
        m_armPauseHigh = false;
        m_armPauseLow = false;
        SmartDashboard.putString("m_setpoint", m_setpoint.toString());
        SmartDashboard.putString("m_queuedSetpoint", m_queuedSetpoint.toString());
        SmartDashboard.putBoolean("m_armPauseHigh", m_armPauseHigh);
        SmartDashboard.putBoolean("m_armPauseLow", m_armPauseLow);
        SmartDashboard.putBoolean("m_elevatorPauseHigh", m_elevatorPauseHigh);
        SmartDashboard.putBoolean("m_elevatorPauseLow", m_elevatorPauseLow);
    }

    // Second constructor defaults to disabled (do nothing until SetState is first called)
    public SSM(Arm arm, Elevator elevator) {
        this(arm, elevator, States.DISABLED);
    }

    // ALways start the elevator if elevator commanded up, only move to the danger point if the final setpoint is beyond.
    // Always start the arm if the elevator commanded down, only move to the danger point if the final setpoint is beyond.
    // kElevatorHighDanger - elevator must be higher than this to safely move arm above kArmHighDanger
    // KElevatorLowDanter - elevator must be higher than this to safely move arm below kArmLowDanger
    private void newState(States state) {
        int m_slewMode = 0;

        m_setpoint = state;
        SmartDashboard.putString("m_setpoint", m_setpoint.toString());

        if (m_setpoint == States.DISABLED) return;

        if (IntakeFromFunnel.CoralHalted && ((m_setpoint == States.L4) || 
            (m_setpoint == States.L3) || (m_setpoint == States.L2) || 
            (m_setpoint == States.L1))) return;

        // Adjust the slew rate based on the setpoint
        switch (m_setpoint) {
            case L4:
            case BARGE:
                if (m_slewMode != 1) DriveCommands.updateSlew(3.4, 3.4, 15.0);
                m_slewMode = 1;
            break;
            default:
                if (m_slewMode !=2) DriveCommands.updateSlew(10.0, 10.0, 30.0);
                m_slewMode = 2;
            break;
        }

        m_as = getScoringArmPosition(m_setpoint);               // Grab some local variables for mulit reuse efficiency
        m_es = getScoringElevatorPosition(m_setpoint);

        // Clear booleans
        m_armPauseHigh = false;
        m_armPauseLow = false;
        m_elevatorPauseHigh = false;
        m_elevatorPauseLow = false;

        if (m_elevator.getPosition() < m_es) {                      // If elevator going up...
            m_elevator.setPosition(m_es);                           // Going up, always start elevator to setpoint
            if ((m_as < ArmConstants.kArmHighDanger) && (m_elevator.getPosition() < ElevatorConstants.kElevatorHighDanger)) {
                m_arm.setPosition(ArmConstants.kArmHighDanger);     // Pause arm at high danger before moving farther up
                m_armPauseHigh = true;
            } else if ((m_as > ArmConstants.kArmLowDanger) && (m_elevator.getPosition() < ElevatorConstants.kElevatorLowDanger)) {
                m_arm.setPosition(ArmConstants.kArmLowDanger);      // Pause arm at low danger before moving farther down
                m_armPauseLow = true;
            } else m_arm.setPosition(m_as);                         // No constraints, move arm to final setpoint
        } else {
            m_arm.setPosition(m_as);                                // Going down, always start arm to setpoint
            if ((m_es < ElevatorConstants.kElevatorHighDanger) && (m_arm.getPosition() < ArmConstants.kArmHighDanger)) {
                m_elevator.setPosition(ElevatorConstants.kElevatorHighDanger);     // Pause elevator at high danger before moving farther down
                m_elevatorPauseHigh = true;
            } else if ((m_es < ElevatorConstants.kElevatorLowDanger) && (m_arm.getPosition() > ArmConstants.kArmLowDanger)) {
                m_elevator.setPosition(ElevatorConstants.kElevatorLowDanger);      // Pause elevator at low danger before moving farther down
                m_elevatorPauseLow = true;
            } else {
                m_elevator.setPosition(m_es);                        // No constraints, move elevator to final setpoint
            }
        }
    }


     public void periodic() {

        if (m_setpoint != m_queuedSetpoint) newState(m_queuedSetpoint);  // Check for new state commanded
        if (m_setpoint == States.DISABLED) return;

        if (m_armPauseHigh && (m_elevator.getPosition() > ElevatorConstants.kElevatorHighDanger)) {           // Elevator going up
            m_arm.setPosition(m_as);                     // Cleared, continue to final setpoint
            m_armPauseHigh = false;
        } else if (m_armPauseLow && (m_elevator.getPosition() > ElevatorConstants.kElevatorLowDanger)) {      // Elevator going up
            m_arm.setPosition(m_as);                     // Cleared, continue to final setpoint
            m_armPauseLow = false;
        } else if (m_elevatorPauseHigh && (m_arm.getPosition() > ArmConstants.kArmHighDanger)) {              // Elevator going down
            m_elevator.setPosition(m_es);                // Cleared, continue to final setpoint
            m_elevatorPauseHigh = false;
        } else if (m_elevatorPauseLow && (m_arm.getPosition() < ArmConstants.kArmLowDanger)) {                // Elevator going down
            m_elevator.setPosition(m_es);                // Cleared, continue to final setpoint
            m_elevatorPauseLow = false;
        }
        SmartDashboard.putBoolean("m_armPauseHigh", m_armPauseHigh);
        SmartDashboard.putBoolean("m_armPauseLow", m_armPauseLow);
        SmartDashboard.putBoolean("m_elevatorPauseHigh", m_elevatorPauseHigh);
        SmartDashboard.putBoolean("m_elevatorPauseLow", m_elevatorPauseLow);
    }

    public void setState(States q) {
        m_queuedSetpoint = q;
        SmartDashboard.putString("m_queuedSetPoint", m_queuedSetpoint.toString());
    }

    private double getScoringElevatorPosition(States state) {
        return switch (state) {
            case L1 -> ElevatorConstants.kElevatorL1;
            case L2 -> ElevatorConstants.kElevatorL2;
            case L3 -> ElevatorConstants.kElevatorL3;
            case L4 -> ElevatorConstants.kElevatorL4;
            case LOADINGSTATION -> ElevatorConstants.kElevatorLoadingStation;
            case PROCESSOR -> ElevatorConstants.kElevatorProcessor;
            case BARGE -> ElevatorConstants.kElevatorBarge;
            case GROUNDALGAE -> ElevatorConstants.kElelvatorGroundAlgae;
            case ALGAEHIGH -> ElevatorConstants.kHighAlgeaGrab;
            case ALGAELOW -> ElevatorConstants.kLowAlgeaGrab;   
            default -> 0.0;
        };
    }

    private double getScoringArmPosition(States state) {
        return switch (state) {
            case L1 -> ArmConstants.kArmL1;
            case L2 -> ArmConstants.kArmL2;
            case L3 -> ArmConstants.kArmL3;
            case L4 -> ArmConstants.kArmL4;
            case LOADINGSTATION -> ArmConstants.kArmFeeder;
            case PROCESSOR -> ArmConstants.kArmProcessor;
            case BARGE -> ArmConstants.kArmBarge;
            case GROUNDALGAE -> ArmConstants.kArmGroundAlgae;
            case ALGAEHIGH -> ArmConstants.kArmAlgeaGrab;
            case ALGAELOW -> ArmConstants.kArmAlgeaGrab;
            default -> 0.0;
        };
    }
}










