package de.uni_luebeck.iti.smachGenerator;


import java.util.HashSet;
import java.util.LinkedList;
import java.util.NoSuchElementException;

public class SmachableActuators extends LinkedList<ISmachableActuator> {


    private static final long serialVersionUID = 4879265984143961788L;

    public ISmachableActuator getActuator(String actuatorName)
            throws NoSuchElementException {
        for (ISmachableActuator actuator : this) {
            if (actuator.getKey().equals(actuatorName))
                return actuator;
        }
        throw new NoSuchElementException("Zugriff auf unbekannten Aktor "
                + actuatorName + ".");
    }

    public HashSet<String> getPublisherSetups() {
        HashSet<String> pubs = new HashSet<String>();
        for (ISmachableActuator actuator : this) {
            pubs.add("pub_" + actuator.getTopic().replace("/", "_") + " = rospy.Publisher('" + actuator.getTopic() + "', "
                    + actuator.getTopicType() + ")");
        }
        return pubs;
    }

    public HashSet<String> getMsgDeps() {
        HashSet<String> deps = new HashSet<String>();
        for (ISmachableActuator actuator : this) {
            deps.add("from " + actuator.getTopicPackage() + " import "
                    + actuator.getTopicType());
        }
        return deps;
    }

}
