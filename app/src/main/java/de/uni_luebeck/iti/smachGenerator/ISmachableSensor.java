package de.uni_luebeck.iti.smachGenerator;

import java.util.Set;

public interface ISmachableSensor {

    /**
     * @return the name of the sensor
     */
    public abstract String getName();

    /**
     * @return a HashSet of all Imports that are needed for this
     * {@link ISmachableSensor}.
     */
    public abstract Set<String> getImports();

    /**
     * Creates the callback function for this sensor. All calculations (if
     * necessary) to be able to compare values for later transitions have to be
     * done here and the result has to be stored in the
     * <code>ValueIdentifier</code>. This <code>ValueIdentifier</code> has to be
     * the same that is initialized by <code>getIdentifierInit()</code> and
     * <code>getGlobalIdentifier</code>.
     *
     * @return the callback for this sensor.
     */
    public abstract String getCallback();

    /**
     * Creates the Subscriber Setup statement. The callback for this Subscriber
     * will be the one that is created by <code>getCallback()</code>.
     *
     * @return Subscriber initialization
     */
    public abstract String getSubscriberSetup();

    /**
     * @return the identifier that stores the current value of this sensor.
     */
    public abstract String getValueIdentifier();

    /**
     * Returns the identifier of the globally defined variable for this sensor.
     * <p>
     * NOTE: this means this might return the name of an Array or anything else.
     * To receive the identifier that stores the current value of this sensor,
     * use <code>getValueIdentifier()</code>!</p>
     * <p><i>May not be empty!</i></p>
     *
     * @return globally identifier name.
     */
    public abstract String getGlobalIdentifier();

    /**
     * <p><i>May not be empty!</i></p>
     *
     * @return statements to define and initialize the value the callback stores
     * values in.
     */
    public abstract String getIdentifierInit();

    /**
     * Returns the condition comparing the current sensor value with the compare
     * value.
     *
     * @param op      Operator to compare the values. Choose one of <, <=, ==, >=, >, !=
     * @param compVal value to compare the current sensor value with.
     * @return a condition representing the comparison of the sensor value with
     * the compare value.
     */
    public abstract String getTransitionCondition(String op, int compVal);

    /**
     * Returns a number of commands that shall be executed before the {@link SmachAutomat} is shut down.
     * Mainly this will be publishing some last messages for the sensor to deactivate etc.
     *
     * @return Some commands to shutdown this sensor
     */
    public abstract String[] onShutDown();

    public String getTopic();

    public String getTopicType();

}
