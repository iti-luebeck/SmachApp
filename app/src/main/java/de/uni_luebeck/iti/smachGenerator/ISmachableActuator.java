package de.uni_luebeck.iti.smachGenerator;

/**
 * An Actuator that can perform Actions.
 */
public interface ISmachableActuator {

    /**
     * Returns the topic to which the control messages should be posted.
     * @return the name of the topic
     */
    public abstract String getTopic();

    /**
     * Returns the key of the Actuator.
     * @return the key
     */
    public abstract String getKey();

    /**
     * The name of the object in the message, for example most std_msgs use data.
     * To create an std_msgs.Int8 message on would write in Python:
     *  <pre>
     * {@code
     * message=new Int8()
     * message.data=42
     * }
     * </pre>
     *Here one can see in the second line, what the object in the message is used for.
     *
     * @return the name of the object in the message
     */
    public abstract String getObjectInMessage();

    /**
     * Returns the type of the topic.
     * @return the type of the topic
     */
    public abstract String getTopicType();

    /**
     * Returns the package that contains the topic type.
     * @return the package of the topic type
     */
    public abstract String getTopicPackage();

}
