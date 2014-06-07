package de.uni_luebeck.iti.smachapp.model;

import de.uni_luebeck.iti.smachGenerator.Operator;

/**
 * Created by Morten Mey on 07.06.2014.
 */
public class IntSensor extends Sensor {
    private int min;
    private int max;


    public IntSensor(String key,String topic,String topicType,String topicPackage,String objectInMessage,int min,int max){
       super(key,topic,topicType,topicPackage,objectInMessage);
        this.min=min;
        this.max=max;
    }

    public int getMax(){
        return max;
    }

    public int getMin(){
        return min;
    }

    public void setMax(int max){
        this.max=max;
    }

    public void setMin(int min) {
        this.min = min;
    }

    @Override
    public String getTransitionCondition(Operator op, int compVal) {
        return key+op+compVal;
    }
}
