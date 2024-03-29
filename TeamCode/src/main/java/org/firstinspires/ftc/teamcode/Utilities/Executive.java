package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * Provides Executive robot Control through class based state machine.
 */
public class Executive {

    /**
     * Interface to be used as t he interface for the class which hosts the
     * state machine object as well as the concrete implementations of the required states.
     */
    public interface RobotStateMachineContextInterface {
        void init();
        void update();
        String getCurrentState();
    }



    /**
     * Robot state machine, supporting multiple named simultaneous states, expanded by adding to an enum.
     */
    static public class StateMachine {

        private Map<StateType, StateBase> stateMap = new HashMap<>();
        RobotHardware opMode;

        /**
         * State Machine will support one additional concurrent state for each possible StateType.
         * Note that StateMachine.remove(StateType.ARM) would remove an existing ARM state, for
         * example.
         */
        public enum StateType {
            DRIVE,
            ARM,
            LIFT,
        }

        public StateMachine(RobotHardware opMode) {
            this.opMode = opMode;
        }

        public void changeState(StateType stateType, StateBase state) {
            stateMap.put(stateType, state);
            state.init(this);
        }

        public void removeStateType(StateType stateType) {
            stateMap.remove(stateType);
        }


        public void clearDeletedStates() {
            for (StateType stateType : StateType.values()) {
                if(stateMap.get(stateType) != null) {
                    if (stateMap.get(stateType).isDeleteRequested()) {
                        stateMap.remove(stateType);
                    }
                }
            }
        }

        public String getCurrentState(StateType stateType) {
            StateBase state = stateMap.get(stateType);
            return state.getClass().toString();
        }

        public String getCurrentState() {
            String stateString = "";
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase state = stateMap.get(type);
                stateString += state + "  ";
            }
            return stateString;
        }

        public void init() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase state = stateMap.get(type);
                if(state != null) {
                    state.init(this);
                }
            }
        }

        /**
         * Check if state is initialized, and throw exception if not.
         * Then, if nextStates has values, StateMachine.add() them and clear new states.
         * Then, if state.isDeleteRequested(), StateMachine.delete(state)
         * otherwise, run state.update()
         */
        public void update() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase state = stateMap.get(type);

                if (!state.isInitialized()) {
                    throw new RuntimeException("ERROR: state called in StateMachine update() was never initialized") ;
                }
                // Delete or update state
                if (state.isDeleteRequested() == false) {
                    state.update();
                }
            }
            clearDeletedStates();
        }


        public void reset() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase state = stateMap.get(type);
                state.reset();
            }
        }


    }


    /**
     * State base class.
     */
    // The class is abstract, but the internal methods are not abstract so that they can be optionally implemented
    static public abstract class StateBase {

        StateMachine stateMachine; // Reference to outer state machine, for modifying states.
        RobotHardware opMode;
        ElapsedTime stateTimer; // Time how long state has been active
        ElapsedTime statePeriod; // Time how long since state has been executed.
        double lastStatePeriod = 0;
        boolean arrived = false;

        private boolean initialized = false;
        private boolean deleteRequested = false;


        public StateBase() {
            // Defining default constructor
            // However, we NEED the state machine reference.
            // Handled by allowing init to take a stateMachine argument.
        }


        public void init(StateMachine stateMachine) {
            this.stateMachine = stateMachine;
            this.opMode = stateMachine.opMode;
            stateTimer = new ElapsedTime();
            statePeriod = new ElapsedTime();
            initialized = true;
            stateTimer.reset();
            statePeriod.reset();
        }

        public void update() {
            lastStatePeriod = statePeriod.seconds();
            statePeriod.reset();
        }

        protected void nextState(StateMachine.StateType stateType, StateBase state) {
            stateMachine.changeState(stateType,state);
            stateMachine.stateMap.get(stateType).init(stateMachine);
        }

        public void reset() {
            initialized = false;
        }

        boolean isInitialized() {
            return initialized;
        }

        public void requestDelete() {
            deleteRequested = true;
        }

        public boolean isDeleteRequested() {
            return deleteRequested;
        }
    }
}
