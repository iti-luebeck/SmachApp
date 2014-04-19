package de.uni_luebeck.iti.smachGenerator;

import java.util.List;

/**
 * A Guard that can be evaluated using the robots sensors and is used to make transitions
 * conditional.
 */
public interface ISmachableGuard {

    /**
     * Returns the name of all sensors a value shall be compared with. Returns
     * an empty List, if there is no condition for this transition.
     *
     * @return List of Sensors
     */
    public abstract List<String> getSensorNames();

    /**
     * Returns a List of {@link Operator}s. Each {@link Operator} belongs to the
     * corresponding sensor in the getSensorNames list.
     *
     * @return List of {@link Operator}s, representing the relation between
     * sensor and compare value
     */
    public abstract List<Operator> getOperators();

    /**
     * Returns a List of {@link Integer}s. Each of these compare values shall be
     * compared with the sensor from the getSensorNames List and the
     * Operator from the getOperators List
     *
     * @return List of values to compare with
     */
    public abstract List<Integer> getCompValues();

}
