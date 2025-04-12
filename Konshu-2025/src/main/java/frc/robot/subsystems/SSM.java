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

    public enum States {DISABLED, L1, L1_IN, L1_INTERIM, L2, L3, L4, LOADINGSTATION, PROCESSOR, BARGE, GROUNDALGAE, ALGAELOW, ALGAEHIGH, Climb};
    // Currently not using all these L1States, but left in case we later want to implement
    // Autoalign while in L1_IN.   This will require extenstions to use all these states AS WELL AS:
    //  1) Update AutoAlign for L1_IN to use PID and set rotation perpendicular to reef (similar to L1 without offset)
    //  2) Set up lookup offsets for arm L1_IN
    //  3) Create new upper and lower limits for arm when inside elevator
    //  4) Add another elseif to initial state change request check to handle when happens while in TO_L1_IN state - would be bad
    //  5) Include L1_IN as valid state for safe move logic so arm/elevator update with offset changes
    private enum L1State {OUT, IN_TO_L1_INTERIM, TO_L1_IN, AT_L1_IN, OUT_TO_L1_INTERIM};

    private boolean m_elevatorPauseHigh, m_elevatorPauseLow, m_armPauseHigh, m_armPauseLow;
    private States m_setpoint, m_queuedSetpoint, m_nextSetpoint;
    private double m_armSetpoint, m_elevatorSetpoint;
    private double m_armOffset, m_elevatorOffset;
    private L1State m_L1state;

    // First constructor accepts a constant as an initial setpoint and starts moving
    // there immediately via periodic
    public SSM(Arm arm, Elevator elevator, States setpoint, BooleanSupplier hasCoral) {
        m_arm = arm;
        m_elevator = elevator;
        m_setpoint = setpoint;
        m_nextSetpoint = setpoint;
        m_queuedSetpoint = setpoint;
        m_hasCoral = hasCoral;
        
        m_L1state = L1State.OUT;
        m_elevatorPauseHigh = false;
        m_elevatorPauseLow = false;
        m_armPauseHigh = false;
        m_armPauseLow = false;
        m_armOffset = 0.0;
        m_elevatorOffset = 0.0;
    }

    // Second constructor defaults to disabled (do nothing until SetState is first
    // called)
    public SSM(Arm arm, Elevator elevator, BooleanSupplier hasCoral) {
        this(arm, elevator, States.DISABLED, hasCoral);
    }

    // ALways start the elevator if elevator commanded up, only move to the danger
    // point if the final setpoint is beyond.
    // Always start the arm if the elevator commanded down, only move to the danger
    // point if the final setpoint is beyond.
    // kElevatorHighDanger - elevator must be higher than this to safely move arm
    // above kArmHighDanger
    // KElevatorLowDanter - elevator must be higher than this to safely move arm
    // below kArmLowDanger
    private void updateState(States state) {

        // Special handling of L1_IN (arm inside of elevator).   Written in protest as both Sam and Emmerson
        // assured me we would never need to move the arm inside the elevator!  The code was so clean before this.
        if (state != m_setpoint) {                             // Check for state change
            if (state == States.L1_IN) {                         // If new state is L1_IN
                state = States.L1_INTERIM;                       //  first go to L_Interim
                m_queuedSetpoint = States.L1_INTERIM;            //  so doesn't reset on next pass
                m_L1state = L1State.IN_TO_L1_INTERIM;            //  and set flag
            } else if (m_setpoint == States.L1_IN) {           // If currently at L1_IN
                m_nextSetpoint = state;                          //  save new commanded state for later
                state = States.L1_INTERIM;                       //  first go to L_Interim
                m_queuedSetpoint = States.L1_INTERIM;            //  so doesn't reset on next pass
                m_L1state = L1State.OUT_TO_L1_INTERIM;           //  and set flag
                m_arm.setPosition(getScoringArmPosition(States.L1_INTERIM));
                m_elevator.setPosition(getScoringElevatorPosition(States.L1_INTERIM));
            }
        }

        m_setpoint = state;
        if (m_setpoint == States.DISABLED) return;

        // Handle special case of L_IN (arm inside of elevator)
        switch (m_L1state) {
            case IN_TO_L1_INTERIM:
                if ((m_elevator.getPosition() > ElevatorConstants.kElevatorL1Interim - 0.5) && 
                    (m_elevator.getPosition() < ElevatorConstants.kElevatorL1Interim + 0.5)) {   
                    m_L1state = L1State.TO_L1_IN;
                    m_arm.setPosition(getScoringArmPosition(States.L1_IN));                 // Now safe to move arm inside elevator
 //                   m_elevator.setPosition(getScoringElevatorPosition(States.L1_IN));     // For now, assume don't move elevator at all when arm inside
                    m_setpoint = States.L1_IN;
                    m_queuedSetpoint = States.L1_IN;
                }
            break;
            case OUT_TO_L1_INTERIM:
                if (m_arm.getPosition() > ArmConstants.kArmHighDanger) {
                    m_L1state = L1State.OUT;
                    m_setpoint = m_nextSetpoint;            // Now safe to move to new commanded state
                    m_queuedSetpoint = m_nextSetpoint;
                }
            break;
            default:
            break;
        }
        
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

        // Safe move logic - only execute if arm not inside elevator 
        if ((m_L1state == L1State.OUT) || (m_L1state == L1State.IN_TO_L1_INTERIM)) {
            m_armSetpoint = getScoringArmPosition(m_setpoint)+m_armOffset;
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
            if (m_hasCoral.getAsBoolean()) updateState(m_queuedSetpoint);
        } else updateState(m_queuedSetpoint);
        
        if (m_setpoint == States.DISABLED) return;

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
            case L1_IN -> ArmConstants.kArmL1IN;
            case L1_INTERIM -> ArmConstants.kArmL1Interim;
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