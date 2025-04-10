package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.commands.DriveCommands;

public class SSM extends SubsystemBase {
    private final Arm m_arm;
    private final Elevator m_elevator;
    private static int m_slewMode = 0;
    private final BooleanSupplier m_hasCoral;

    public enum States {
        DISABLED, L1, L1_IN, L1_INTERIM, L2, L3, L4, LOADINGSTATION, PROCESSOR, BARGE, GROUNDALGAE, ALGAELOW, ALGAEHIGH, Climb
    };

    private boolean m_elevatorPauseHigh, m_elevatorPauseLow, m_armPauseHigh, m_armPauseLow, m_pauseSpecial;
    private States m_setpoint, m_queuedSetpoint, m_nextSetpoint;
    private double m_armSetpoint, m_elevatorSetpoint;
    private double m_armOffset, m_elevatorOffset;

    // First constructor accepts a constant as an initial setpoint and starts moving
    // there immediately via periodic
    public SSM(Arm arm, Elevator elevator, States setpoint, BooleanSupplier hasCoral) {
        m_arm = arm;
        m_elevator = elevator;
        m_setpoint = States.DISABLED;
        m_nextSetpoint = setpoint;
        m_queuedSetpoint = setpoint;
        m_elevatorPauseHigh = false;
        m_elevatorPauseLow = false;
        m_armPauseHigh = false;
        m_armPauseLow = false;
        m_pauseSpecial = false;
        SmartDashboard.putString("m_queuedSetpoint", m_queuedSetpoint.toString());
        SmartDashboard.putBoolean("m_armPauseHigh", m_armPauseHigh);
        SmartDashboard.putBoolean("m_armPauseLow", m_armPauseLow);
        SmartDashboard.putBoolean("m_elevatorPauseHigh", m_elevatorPauseHigh);
        SmartDashboard.putBoolean("m_elevatorPauseLow", m_elevatorPauseLow);

        m_armOffset = 0.0;
        m_elevatorOffset = 0.0;

        m_hasCoral = hasCoral;
    }

    // Second constructor defaults to disabled (do nothing until SetState is first
    // called)
    public SSM(Arm arm, Elevator elevator, BooleanSupplier hasCoral) {
        this(arm, elevator, States.DISABLED,hasCoral);
    }

    // ALways start the elevator if elevator commanded up, only move to the danger
    // point if the final setpoint is beyond.
    // Always start the arm if the elevator commanded down, only move to the danger
    // point if the final setpoint is beyond.
    // kElevatorHighDanger - elevator must be higher than this to safely move arm
    // above kArmHighDanger
    // KElevatorLowDanter - elevator must be higher than this to safely move arm
    // below kArmLowDanger
    private void newState(States state) {

        SmartDashboard.putString("state in", state.toString());
        SmartDashboard.putString("m_setpoint in", m_setpoint.toString());
        SmartDashboard.putString("m_queuedSetpoint in", m_queuedSetpoint.toString());
        SmartDashboard.putBoolean("m_pauseSpecial in", m_pauseSpecial);

        // Special handling of L1_IN (arm inside of elevator)
        if (state != m_setpoint) {                             // Check for state change
            if (state == States.L1_IN) {                        // If new state is L1_IN
                state = States.L1_INTERIM;                      //  first go to L_Interim
                m_queuedSetpoint = States.L1_INTERIM;
                m_pauseSpecial = true;                          //  and set flag
            } else if (m_setpoint == States.L1_IN) {            // If currently at L1_IN
                m_nextSetpoint = state;                         //  save commanded state for later
                m_queuedSetpoint = States.L1_INTERIM;
                state = States.L1_INTERIM;                     //  and first go to L1_INTERIM
                m_pauseSpecial = true;                         //  and set flag
            }
        }
        SmartDashboard.putBoolean("m_pauseSpecial out 1", m_pauseSpecial);
        SmartDashboard.putString("m_nextSetpoint", m_nextSetpoint.toString());


        m_setpoint = state;
        SmartDashboard.putString("m_setpoint out", m_setpoint.toString());

        if (m_setpoint == States.DISABLED)
            return;

        // Adjust the slew rate based on the setpoint
        switch (m_setpoint) {
            case L3:
                if (m_slewMode != 1) {
                    DriveCommands.updateSlew(7, 7, 36.0);
                    m_slewMode = 1;
                }
            break;
            case L4:
            case BARGE:
                if (m_slewMode != 2) {
                    DriveCommands.updateSlew(4.0, 4.0, 36.0);
                    m_slewMode = 2;
                }
            break;
            default:
                if (m_slewMode != 3) {
                    DriveCommands.updateSlew(12.0, 12.0, 36.0);
                    m_slewMode = 3;
                }
            break;
        }

        m_armSetpoint = getScoringArmPosition(m_setpoint)+m_armOffset; // Grab some local variables for mulit reuse efficiency
        m_armSetpoint = Math.max(m_armSetpoint, ArmConstants.kArmUpperLimit);
        m_armSetpoint = Math.min(m_armSetpoint, ArmConstants.kArmLowerLimit); 

        m_elevatorSetpoint = getScoringElevatorPosition(m_setpoint)+m_elevatorOffset;
        m_elevatorSetpoint = Math.min(m_elevatorSetpoint, ElevatorConstants.kUpperLimitElevator);
        m_elevatorSetpoint = Math.max(m_elevatorSetpoint, ElevatorConstants.kULowerLimitElevator);

        // Clear booleans
        m_armPauseHigh = false;
        m_armPauseLow = false;
        m_elevatorPauseHigh = false;
        m_elevatorPauseLow = false;

        // Handle special case of L_IN (arm inside of elevator)
        if (m_pauseSpecial) {
            if (m_setpoint == States.L1_INTERIM) {
                if ((m_elevator.getPosition() > m_elevatorSetpoint - 0.5) && (m_elevator.getPosition() < m_elevatorSetpoint + 0.5)) {    // Moving to L1_INTERIM in prep to move to L1_IN - now safe to move arm inside elevator
                    m_setpoint = States.L1_IN;
                    m_queuedSetpoint = States.L1_IN;
                    m_arm.setPosition(getScoringArmPosition(m_setpoint));
                    m_elevator.setPosition(getScoringElevatorPosition(m_setpoint));
                    m_pauseSpecial = false;
                }
            } else if (m_arm.getPosition() > ArmConstants.kArmHighDanger) {     // Moving to L1_INTERIM from L1_IN - now safe to move to final state
                m_setpoint = m_nextSetpoint;
                m_queuedSetpoint = m_nextSetpoint;
                m_pauseSpecial = false;
                return;
            }
        }
        SmartDashboard.putBoolean("m_pauseSpecial out 2", m_pauseSpecial);

        if (m_setpoint != States.L1_IN) {
            if (m_elevator.getPosition() < m_elevatorSetpoint) { // If elevator going up...
               m_elevator.setPosition(m_elevatorSetpoint); // Going up, always start elevator to setpoint
               if ((m_armSetpoint < ArmConstants.kArmHighDanger)
                       && (m_elevator.getPosition() < ElevatorConstants.kElevatorHighDanger)) {
                   m_arm.setPosition(ArmConstants.kArmHighDanger); // Pause arm at high danger before moving farther up
                   m_armPauseHigh = true;
               } else if ((m_armSetpoint > ArmConstants.kArmLowDanger)
                       && (m_elevator.getPosition() < ElevatorConstants.kElevatorLowDanger)) {
                   m_arm.setPosition(ArmConstants.kArmLowDanger); // Pause arm at low danger before moving farther down
                   m_armPauseLow = true;
               } else
                    m_arm.setPosition(m_armSetpoint); // No constraints, move arm to final setpoint
            } else {
                m_arm.setPosition(m_armSetpoint); // Going down, always start arm to setpoint
                if ((m_elevatorSetpoint < ElevatorConstants.kElevatorHighDanger) && (m_arm.getPosition() < ArmConstants.kArmHighDanger)) {
                    m_elevator.setPosition(ElevatorConstants.kElevatorHighDanger); // Pause elevator at high danger before moving farther down
                    m_elevatorPauseHigh = true;
                } else if ((m_elevatorSetpoint < ElevatorConstants.kElevatorLowDanger)
                       && (m_arm.getPosition() > ArmConstants.kArmLowDanger)) {
                   m_elevator.setPosition(ElevatorConstants.kElevatorLowDanger); // Pause elevator at low danger before moving farther down
                   m_elevatorPauseLow = true;
               } else
                   m_elevator.setPosition(m_elevatorSetpoint); // No constraints, move elevator to final setpoint
            }
        }
    }

    public void periodic() {
        /// SAFETY CHECK - Make sure the arm is not inside of the elevator (start position)
        /// Only check if disabled (at start) now that arm CAN move inside elevator for L1_IN
        if ((m_setpoint == States.DISABLED) && 
         (m_elevator.getPosition() < ElevatorConstants.kElevatorHighDanger) &&
         (m_arm.getPosition() < ArmConstants.kArmHighDanger)) {
            m_arm.setPosition(getScoringArmPosition(States.LOADINGSTATION));
            return;
        }

//        if (m_setpoint != m_queuedSetpoint)      // Now always reinit because added offsets
        if ((m_queuedSetpoint == States.L1) || (m_queuedSetpoint == States.L2) || 
            (m_queuedSetpoint == States.L3) || (m_queuedSetpoint == States.L4) ||
            (m_queuedSetpoint == States.L1_IN)) {
            if (m_hasCoral.getAsBoolean()) newState(m_queuedSetpoint);
        } else newState(m_queuedSetpoint);                // Check for new state commanded
        
        if (m_setpoint == States.DISABLED)
            return;

        
        if (m_armPauseHigh && (m_elevator.getPosition() > ElevatorConstants.kElevatorHighDanger)) { // Elevator going up
            m_arm.setPosition(m_armSetpoint); // Cleared, continue to final setpoint
            m_armPauseHigh = false;
        } else if (m_armPauseLow && (m_elevator.getPosition() > ElevatorConstants.kElevatorLowDanger)) { // Elevator going up
            m_arm.setPosition(m_armSetpoint); // Cleared, continue to final setpoint
            m_armPauseLow = false;
        } else if (m_elevatorPauseHigh && (m_arm.getPosition() > ArmConstants.kArmHighDanger)) { // Elevator going down
            m_elevator.setPosition(m_elevatorSetpoint); // Cleared, continue to final setpoint
            m_elevatorPauseHigh = false;
        } else if (m_elevatorPauseLow && (m_arm.getPosition() < ArmConstants.kArmLowDanger)) { // Elevator going down
            m_elevator.setPosition(m_elevatorSetpoint); // Cleared, continue to final setpoint
            m_elevatorPauseLow = false;
        }
        SmartDashboard.putBoolean("m_armPauseHigh", m_armPauseHigh);
        SmartDashboard.putBoolean("m_armPauseLow", m_armPauseLow);
        SmartDashboard.putBoolean("m_elevatorPauseHigh", m_elevatorPauseHigh);
        SmartDashboard.putBoolean("m_elevatorPauseLow", m_elevatorPauseLow);
    }

    public void setState(States q) {
        m_queuedSetpoint = q;
        SmartDashboard.putString("m_queuedSetpoint", m_queuedSetpoint.toString());
        m_armOffset = 0.0;
        m_elevatorOffset = 0.0;
    }

    public void setState(States q, double armOffset, double elevatorOffset) {
        m_queuedSetpoint = q;
        SmartDashboard.putString("m_queuedSetpoint", m_queuedSetpoint.toString());
        m_armOffset = armOffset;
        m_elevatorOffset = elevatorOffset;
    }

    public States getState(){
        return m_setpoint;
    }

    public double getScoringElevatorPosition(States state) {
        return switch (state) {
            case L1 -> ElevatorConstants.kElevatorL1;
            case L1_IN -> ElevatorConstants.kElevatorL1IN;
            case L1_INTERIM -> ElevatorConstants.kElevatorL1Interim;
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

    public double getScoringArmPosition(States state) {
        return switch (state) {
            case L1 -> ArmConstants.kArmL1;
            case L1_IN -> ArmConstants.KArmL1IN;
            case L1_INTERIM -> ArmConstants.KArmL1Interim;
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