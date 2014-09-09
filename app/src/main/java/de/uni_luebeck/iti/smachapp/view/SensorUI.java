package de.uni_luebeck.iti.smachapp.view;

import de.uni_luebeck.iti.smachapp.model.Guard;

/**
 * Created by Morten Mey on 07.06.2014.
 */
public interface SensorUI {

    public abstract void setToGuard(Guard guard,boolean checked);

    public abstract void fillGuard(Guard guard);

    public abstract boolean isChecked();
}
