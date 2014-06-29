package de.uni_luebeck.iti.smachapp.model;

import android.graphics.Color;

import java.util.HashSet;
import java.util.Set;

import de.uni_luebeck.iti.smachGenerator.ISmachableAction;
import de.uni_luebeck.iti.smachGenerator.ISmachableActuator;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class BeepColorRGBActuator implements ISmachableActuator {

    private int defaultColor = Color.RED;

    private String name;
    private String topic;
    private final int ledIndex;

    public BeepColorRGBActuator(String name, String topic, int ledIndex) {
        this.name = name;
        this.topic = topic;
        this.ledIndex = ledIndex;
    }

    public BeepColorRGBActuator(String name, String topic, int ledIndex, int defaultColor) {
        this(name, topic, ledIndex);
        this.defaultColor = defaultColor;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String getPublisherSetup() {
        return "pub_led = rospy.Publisher('" + topic + "', Led)";
    }

    @Override
    public String getPublisherName() {
        return "pub_led";
    }

    @Override
    public String[] getPublishMessage(ISmachableAction a) {
        String[] res = new String[10];
        String col = "c" + ledIndex;
        String led = "led" + ledIndex;

        res[0] = col + " = MyColor()";
        res[1] = col + ".r = " + Color.red(a.getValue());
        res[2] = col + ".g = " + Color.green(a.getValue());
        res[3] = col + ".b = " + Color.blue(a.getValue());
        res[4] = led + " = Led()";
        res[5] = led + ".header.frame_id = 'led'";
        res[6] = led + ".header.stamp = rospy.get_rostime()";
        res[7] = led + ".col = " + col;
        res[8] = led + ".led = " + ledIndex;
        res[9] = getPublisherName() + ".publish(" + led + ")";

        return res;
    }

    @Override
    public Set<String> getImports() {
        HashSet<String> res = new HashSet<String>();
        res.add("from beep_msgs.msg import Led");
        res.add("from beep_msgs.msg import MyColor");
        return res;
    }

    @Override
    public String[] onShutDown() {
        Action stoppen = new Action("color", 0);
        return getPublishMessage(stoppen);
    }

    public void setDefaultColor(int color) {
        defaultColor = color;
    }

    public int getDefaultColor() {
        return defaultColor;
    }
}
