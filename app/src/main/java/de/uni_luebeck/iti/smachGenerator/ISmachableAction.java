package de.uni_luebeck.iti.smachGenerator;

/**
 * An Action that can be executed by an {@link de.uni_luebeck.iti.smachGenerator.ISmachableActuator}.
 */
public interface ISmachableAction {

    /**
     * Returns the key of the Actuator for this Action.
     *
     * @return the key of the Actuator
     */
    public abstract String getKey();

    /**
     * Returns the value for the Actuator.
     *
     * @return the value
     */
    public abstract int getValue();

}