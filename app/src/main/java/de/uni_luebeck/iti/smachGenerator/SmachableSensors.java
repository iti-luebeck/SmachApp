package de.uni_luebeck.iti.smachGenerator;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.NoSuchElementException;

public class SmachableSensors extends LinkedList<ISmachableSensor> {

    private static final long serialVersionUID = -7460092781520543805L;


    public ISmachableSensor getSensor(String sensorName)
            throws NoSuchElementException {
        for (ISmachableSensor sensor : this) {
            if (sensor.getKey().equals(sensorName))
                return sensor;
        }
        throw new NoSuchElementException("Zugriff auf unbekannten Sensor "
                + sensorName + ".");
    }

    public LinkedList<String> getCallbacks() throws IllegalArgumentException {
        HashMap<String, String> callbacks = new HashMap<String, String>();
        for (ISmachableSensor sensor : this) {
            String cb;
            if (callbacks.containsKey(sensor.getTopic())) {
                cb = "\tglobal " + sensor.getKey() + "\n\t" + sensor.getKey()
                        + " = msg." + sensor.getObjectInMessage() + "\n";
                if (callbacks.get(sensor.getTopic()).contains(
                        "\t" + sensor.getKey() + " = msg.")
                        || callbacks.get(sensor.getTopic()).contains(
                        " = msg." + sensor.getObjectInMessage() + "\n")) {
                    throw new IllegalArgumentException(
                            sensor.getKey()
                                    + " is not unique. It has the same name as an other sensor or cannot be differentiated by topic and objectInMessage from an other Sensor"
                    );
                }
                cb = callbacks.get(sensor.getTopic()) + cb;
            } else {
                cb = "def callback_" + sensor.getTopic().replace("/", "_")
                        + "(msg):\n" + "\tglobal " + sensor.getKey() + "\n\t"
                        + sensor.getKey() + " = msg."
                        + sensor.getObjectInMessage() + "\n";

            }
            callbacks.put(sensor.getTopic(), cb);
        }
        LinkedList<String> results = new LinkedList<String>();
        for (String cb : callbacks.values()) {
            results.add(cb);
        }
        return results;
    }

    public HashSet<String> getSubscriberSetups() {
        HashSet<String> subs = new HashSet<String>();
        for (ISmachableSensor sensor : this) {
            subs.add("rospy.Subscriber('" + sensor.getTopic() + "', "
                    + sensor.getTopicType() + ", callback_"
                    + sensor.getTopic().replace("/", "_") + ")\n");
        }
        return subs;
    }

    public HashSet<String> getMsgDeps() {
        HashSet<String> deps = new HashSet<String>();
        for (ISmachableSensor sensor : this) {
            deps.add("from " + sensor.getTopicPackage() + " import "
                    + sensor.getTopicType());
        }
        return deps;
    }

}
