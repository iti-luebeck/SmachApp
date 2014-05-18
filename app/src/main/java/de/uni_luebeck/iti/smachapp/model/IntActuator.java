package de.uni_luebeck.iti.smachapp.model;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class IntActuator extends Actuator {

    private int min=Integer.MIN_VALUE;
    private int max=Integer.MAX_VALUE;

    public IntActuator(String key, String topic, String topicType, String topicPackage, String objectInMessage) {
        super(key, topic, topicType, topicPackage, objectInMessage);
    }

    public IntActuator(String key, String topic, String topicType, String topicPackage, String objectInMessage,int min,int max) {
        super(key, topic, topicType, topicPackage, objectInMessage);
        this.min=min;
        this.max=max;
    }

    public void setMin(int min){
        this.min=min;
    }

    public void setMax(int max){
        this.max=max;
    }

    public int getMin(){
        return min;
    }

    public int getMax(){
        return max;
    }
}
