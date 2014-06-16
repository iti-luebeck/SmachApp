package de.uni_luebeck.iti.smachGenerator;

public interface ISmachableAction {

    /**
     * returns the name of the {@link ISmachableActuator} that shall perform an
     * action
     *
     * @return name of the {@link ISmachableActuator}
     */
    public abstract String getActuatorName();

    /**
     * returns the value the {@link ISmachableActuator} shall interpret as an
     * action for it to do.
     *
     * @return value representing the action
     */
    public abstract int getValue();

}