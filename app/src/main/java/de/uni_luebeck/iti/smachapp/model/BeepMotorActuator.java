package de.uni_luebeck.iti.smachapp.model;

import java.util.HashSet;

import de.uni_luebeck.iti.smachGenerator.ISmachableAction;
import de.uni_luebeck.iti.smachGenerator.ISmachableActuator;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class BeepMotorActuator implements ISmachableActuator {

    private int min = -128;
    private int max = 127;

    private String name;
    private String topic;
    private final String topicType = "Int8";

    public BeepMotorActuator(String name, String topic) {
        this.name = name;
        this.topic = topic;
    }


    @Override
    public String getName() {
        return name;
    }

    @Override
    public String getPublisherSetup() {
        return getPublisherName() + " = rospy.Publisher('" + topic
                + "', " + topicType + ")";
    }

    @Override
    public String[] getPublishMessage(ISmachableAction a) {
        String[] result = new String[3];
        result[0] = name + " = " + topicType + "()";
        result[1] = name + ".data = " + a.getValue();
        result[2] = getPublisherName() + ".publish(" + name + ")";
        return result;
    }

    @Override
    public HashSet<String> getImports() {
        HashSet<String> result = new HashSet<String>();
        result.add("from std_msgs.msg import " + topicType);
        return result;
    }

    @Override
    public String getPublisherName() {
        return "pub_" + name;
    }

    @Override
    public String[] onShutDown() {
        String[] commands = new String[1];
        commands[0] = getPublisherName() + ".publish(0)";
        return commands;
    }

    public void setMin(int min) {
        this.min = min;
    }

    public void setMax(int max) {
        this.max = max;
    }

    public int getMin() {
        return min;
    }

    public int getMax() {
        return max;
    }
}
