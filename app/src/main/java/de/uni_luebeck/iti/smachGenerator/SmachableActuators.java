package de.uni_luebeck.iti.smachGenerator;


import java.util.HashSet;
import java.util.LinkedList;
import java.util.NoSuchElementException;

public class SmachableActuators extends LinkedList<ISmachableActuator> {

    private static final long serialVersionUID = 4879265984143961788L;

    /**
     * Searches for the actuator and returns the first
     * {@link ISmachableActuator} instance with this name.
     *
     * @param actuatorName
     * @return first {@link ISmachableActuator} with the specified name
     * @throws NoSuchElementException if there is no matching {@link ISmachableActuator} found in
     *                                this list.
     */
    public ISmachableActuator getActuator(String actuatorName)
            throws NoSuchElementException {
        for (ISmachableActuator actuator : this) {
            if (actuator.getName().equals(actuatorName))
                return actuator;
        }
        throw new NoSuchElementException("Zugriff auf unbekannten Aktor "
                + actuatorName + ".");
    }

    /**
     * Creates Strings representing the definition of all publishers that are
     * needed to communicate with all actuators stored in this instance. Name of
     * the publisher for a certain topic is "pub_<code>topic</code>
     * ", where all "/" in <code>tobic</code> will be replaced by "_".
     * <p/>
     * Example: to publish something on the topic <code>abc/def</code> you will
     * have to use the publisher <code>pub_abc_def</code>.
     *
     * @return {@link HashSet} of publisher definitions
     */
    public HashSet<String> getPublisherSetups() {
        HashSet<String> pubs = new HashSet<String>();
        for (ISmachableActuator actuator : this) {
            pubs.add(actuator.getPublisherSetup());
        }
        return pubs;
    }

    /**
     * Creates Strings representing all message imports that are needed to
     * communicate with all {@link ISmachableActuator} in this instance
     *
     * @return {@link HashSet} of all message imports
     */
    public HashSet<String> getMsgDeps() {
        HashSet<String> deps = new HashSet<String>();
        for (ISmachableActuator actuator : this) {
            deps.addAll(actuator.getImports());
        }
        return deps;
    }

    /**
     * returns a HashSet of publisher names of all {@link ISmachableActuator}s
     * stored in this List. These are used to declare the identifiers global in
     * certain functions.
     *
     * @return HashSet of publisherNames of the {@link ISmachableActuator}s
     */
    public HashSet<String> getGlobalIdentifiers() {
        HashSet<String> res = new HashSet<String>();
        for (ISmachableActuator actuator : this) {
            res.add(actuator.getPublisherName());
        }
        return res;
    }
}