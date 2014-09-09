package de.uni_luebeck.iti.smachapp.view;

import de.uni_luebeck.iti.smachapp.model.Action;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public interface ActuatorUI {

    public abstract void setToAction(Action action,boolean checked);

    public abstract Action createAction();

    public abstract boolean isChecked();
}
