package de.uni_luebeck.iti.smachGenerator;

/**
 * One transition from one state to another.
 */
public interface ISmachableTransition {

    /**
     * Returns the state the state machine is in after this transition.
     * @return the following state
     */
    public abstract ISmachableState getFollowerState();

    /**
     * Returns the guard for this transition. This must always return a guard, even if the
     * transition is unconditional, in that case the guard should be empty.
     * @see de.uni_luebeck.iti.smachGenerator.ISmachableGuard
     * @return the guard for this transition
     */
    public abstract ISmachableGuard getSmachableGuard();

    /**
     * Returns the label of this transition.
     * @return the label
     */
    public abstract String getLabel();

}