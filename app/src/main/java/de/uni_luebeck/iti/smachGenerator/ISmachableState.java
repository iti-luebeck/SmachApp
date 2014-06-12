package de.uni_luebeck.iti.smachGenerator;

import java.util.List;

/**
 * One state of the state machine.
 */
public interface ISmachableState {
    /**
     * Returns a List containing all actions to be executed in this state.
     *
     * @return a list of actions
     */
    public abstract List<? extends ISmachableAction> getActions();

    /**
     * Returns a List of all transitions from this state.
     *
     * @return a List of transitions
     */
    public abstract List<? extends ISmachableTransition> getTransitions();

    /**
     * Returns true if this is the initial state, there may only be one initial state.
     *
     * @return true if this is the initial state
     */
    public abstract boolean isInitialState();

    /**
     * Returns the name of this state.
     *
     * @return the name of the state
     */
    public abstract String getName();

}