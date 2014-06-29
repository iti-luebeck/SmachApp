package de.uni_luebeck.iti.smachapp.model.undo;

import java.util.ArrayList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.Guard;
import de.uni_luebeck.iti.smachapp.model.Transition;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class TransitionPropertyChanged implements UndoableAction {

    private class GuardElement {
        public String sensor;
        public String op;
        public int value;

        public GuardElement(String sensor, String op, int value) {
            this.sensor = sensor;
            this.op = op;
            this.value = value;
        }

    }

    private Transition transition;

    private String oldName;
    private String newName;

    private List<GuardElement> oldGuard;
    private List<GuardElement> newGuard;

    public TransitionPropertyChanged(Transition transition) {
        this.transition = transition;

        oldName = transition.getLabel();

        Guard g = transition.getSmachableGuard();
        oldGuard = new ArrayList<GuardElement>(g.getCompValues().size());
        for (int i = 0; i < g.getOperators().size(); i++) {
            oldGuard.add(new GuardElement(g.getSensorNames().get(i), g.getOperators().get(i), g.getCompValues().get(i)));
        }
    }

    public boolean operationDone() {

        newName = transition.getLabel();

        Guard g = transition.getSmachableGuard();
        newGuard = new ArrayList<GuardElement>(g.getCompValues().size());
        for (int i = 0; i < g.getOperators().size(); i++) {
            newGuard.add(new GuardElement(g.getSensorNames().get(i), g.getOperators().get(i), g.getCompValues().get(i)));
        }

        return true;
    }

    @Override
    public void undo() {
        transition.setLabel(oldName);
        Guard g = transition.getSmachableGuard();
        g.clear();

        for (GuardElement elem : oldGuard) {
            g.add(elem.sensor, elem.op, elem.value);
        }
    }

    @Override
    public void redo() {
        transition.setLabel(newName);
        Guard g = transition.getSmachableGuard();
        g.clear();

        for (GuardElement elem : newGuard) {
            g.add(elem.sensor, elem.op, elem.value);
        }

    }
}
