package de.uni_luebeck.iti.smachapp.model;

import de.uni_luebeck.iti.smachGenerator.ISmachableAction;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class Action implements ISmachableAction {

    private String actuatorName;
    private int value;

    public Action(String key, int value) {
        this.actuatorName = key;
        this.value = value;
    }

    @Override
    public String getActuatorName() {
        return actuatorName;
    }

    @Override
    public int getValue() {
        return value;
    }

    public void setActuatorName(String actuatorName) {
        this.actuatorName = actuatorName;
    }

    public void setValue(int value) {
        this.value = value;
    }
}
