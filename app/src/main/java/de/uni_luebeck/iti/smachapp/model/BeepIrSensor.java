package de.uni_luebeck.iti.smachapp.model;

import java.util.HashSet;
import java.util.Set;

import de.uni_luebeck.iti.smachGenerator.ISmachableSensor;

/**
 * Created by Morten Mey on 07.06.2014.
 */
public class BeepIRSensor implements ISmachableSensor {
    private int min;
    private int max;

    private String name;
    private String topic;
    private final int irIndex;
    private final String topicType = "beep_msgs/IR";


    public BeepIRSensor(String name, String topic, int index, int min, int max) {
        this.min = min;
        this.max = max;
        this.name = name;
        this.topic = topic;
        this.irIndex = index;
    }

    public BeepIRSensor(String name, String topic, int index) {
        this(name, topic, index, 0, 255);
    }

    public int getMax() {
        return max;
    }

    public int getMin() {
        return min;
    }

    public void setMax(int max) {
        this.max = max;
    }

    public void setMin(int min) {
        this.min = min;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Set<String> getImports() {
        String temp[] = topicType.split("/");
        HashSet<String> res = new HashSet<String>();
        res.add("from " + temp[0] + ".msg import " + temp[1]);
        return res;
    }

    @Override
    public String getCallback() {
        String res = "";
        res += "def ir_cb(msg):\n";
        res += "\tglobal ir\n";
        res += "\tir = msg.ir\n";
        return res;
    }

    @Override
    public String getSubscriberSetup() {
        return "rospy.Subscriber('" + topic + "', " + topicType.split("/")[1]
                + ", ir_cb)\n";
    }

    @Override
    public String getValueIdentifier() {
        return "ir[" + irIndex + "]";
    }

    @Override
    public String getGlobalIdentifier() {
        return "ir";
    }

    @Override
    public String getIdentifierInit() {
        return "ir = [0, 0, 0, 0, 0, 0, 0, 0]";
    }

    @Override
    public String getTransitionCondition(String op, int compVal) {
        return getValueIdentifier() + op + compVal;
    }

    @Override
    public String[] onShutDown() {
        return new String[0];
    }
}
