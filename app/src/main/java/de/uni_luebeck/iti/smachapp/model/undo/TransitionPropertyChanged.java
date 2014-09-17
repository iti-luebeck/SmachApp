package de.uni_luebeck.iti.smachapp.model.undo;

import android.content.Context;
import android.widget.Toast;

import java.util.Hashtable;
import java.util.List;
import java.util.Map;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.Guard;
import de.uni_luebeck.iti.smachapp.model.Transition;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class TransitionPropertyChanged implements UndoableAction {

    private class GuardElement {
        public String op;
        public Integer value;

        public GuardElement(String op, Integer value) {
            this.op = op;
            this.value = value;
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof GuardElement)) {
                return false;
            }

            GuardElement elem = (GuardElement) other;

            return op.equals(elem.op) && value.equals(elem.value);
        }

    }

    private Context context;

    private Transition transition;
    private List<Transition> parentList;
    private int oldPriority;
    private int newPriority;

    private String oldName;
    private String newName;

    private Hashtable<String, GuardElement> oldGuard = new Hashtable<String, GuardElement>();
    private Hashtable<String, GuardElement> newGuard = new Hashtable<String, GuardElement>();

    public TransitionPropertyChanged(Transition transition, List<Transition> parent, int priority,Context context) {
        this.context=context;
        this.transition = transition;
        parentList = parent;
        oldPriority = priority;
        oldName = transition.getLabel();

        Guard g = transition.getSmachableGuard();
        for (int i = 0; i < g.getOperators().size(); i++) {
            oldGuard.put(g.getSensorNames().get(i), new GuardElement(g.getOperators().get(i), g.getCompValues().get(i)));
        }
    }

    public boolean operationDone(int priority) {
        newPriority = priority;
        newName = transition.getLabel();

        boolean change = !oldName.equals(newName);

        Guard g = transition.getSmachableGuard();
        for (int i = 0; i < g.getOperators().size(); i++) {
            String key = g.getSensorNames().get(i);
            GuardElement elem = new GuardElement(g.getOperators().get(i), g.getCompValues().get(i));
            newGuard.put(key, elem);

            if (!(oldGuard.containsKey(key) && oldGuard.get(key).equals(elem))) {
                change = true;
            }
        }

        return change || didPriorityChange();
    }

    @Override
    public void undo() {
        if(didPriorityChange()) {
            parentList.remove(transition);
            parentList.add(oldPriority, transition);
        }
        transition.setLabel(oldName);
        Guard g = transition.getSmachableGuard();
        g.clear();

        for (Map.Entry<String, GuardElement> elem : oldGuard.entrySet()) {
            g.add(elem.getKey(), elem.getValue().op, elem.getValue().value);
        }

        Toast.makeText(context, R.string.undid_transition_properties,Toast.LENGTH_SHORT).show();
    }

    @Override
    public void redo() {
        if(didPriorityChange()) {
            parentList.remove(transition);
            parentList.add(newPriority, transition);
        }
        transition.setLabel(newName);
        Guard g = transition.getSmachableGuard();
        g.clear();

        for (Map.Entry<String, GuardElement> elem : newGuard.entrySet()) {
            g.add(elem.getKey(), elem.getValue().op, elem.getValue().value);
        }

        Toast.makeText(context,R.string.redid_transition_properties,Toast.LENGTH_SHORT).show();
    }

    public  boolean didPriorityChange(){
        return newPriority!=oldPriority;
    }

    public Transition getTransition(){
        return transition;
    }
}
