package de.uni_luebeck.iti.smachGenerator;


import java.util.Set;

public interface ISmachableActuator {

    /**
     * @return the name of the sensor
     */
    public abstract String getName();

    /**
     * Creates the publisher setup statement. These definitions will be done
     * globally. The name of the publisher will be the same as
     * <code>getPublisherName()</code>.
     *
     * @return Publisher initialization
     */
    public abstract String getPublisherSetup();

    /**
     * @return the name of the publisher.
     */
    public abstract String getPublisherName();

    /**
     * Creates all statements that are needed to create a ros message for this
     * {@link ISmachableActuator} and to publish it. Add all statements in
     * ascending oder to publish the message.
     *
     * @param a {@link ISmachableAction} that represents the command and the
     *          content of the message.
     * @return a ordered amount of statements to publish a message for this
     * {@link ISmachableActuator}
     */
    public abstract String[] getPublishMessage(ISmachableAction a);

    /**
     * @return a HashSet of all Imports that are needed for this
     * {@link ISmachableActuator}.
     */
    public abstract Set<String> getImports();


    /**
     * Returns a number of commands that shall be executed before the {@link SmachAutomat} is shut down.
     * Mainly this will be publishing some last messages for the actuator to deactivate etc.
     *
     * @return Some commands to shutdown this actuator
     */
    public abstract String[] onShutDown();

}
