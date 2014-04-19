package de.uni_luebeck.iti.smachGenerator;

/**
 * A Sensor of the robot that can be used to evaluate Guards.
 */
public interface ISmachableSensor {

    /**
     * Returns the name of the sensors topic.
     * @return the name of the topic
     */
    public abstract String getTopic();

    /**
     * Returns the key of this sensor.
     * @return the key
     */
    public abstract String getKey();

    /**
     * Returns the object in the message.
     * @see de.uni_luebeck.iti.smachGenerator.ISmachableActuator
     * @return the object in the message
     */
    public abstract String getObjectInMessage();

    /**
     * Returns the type of this sensors topic.
     * @return the type of the topic
     */
    public abstract String getTopicType();

    /**
     * Returns the package that contains the topic type.
     * @return the package
     */
    public abstract String getTopicPackage();

    /**
     * Returns a valid Pyhton boolean expression, that evaluates to true, if the condition indicated
     * by op and CompVal is true.
     * @param op the operator to use
     * @param compVal the value to which the current sensor value should be compared
     * @return a valid Python boolean expression
     */
    public abstract String getTransitionCondition(Operator op, int compVal);

}
