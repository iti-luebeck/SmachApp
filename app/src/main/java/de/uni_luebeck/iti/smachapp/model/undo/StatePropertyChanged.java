package de.uni_luebeck.iti.smachapp.model.undo;

import java.util.Hashtable;
import java.util.Map;

import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.StateMachine;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class StatePropertyChanged implements UndoableAction {

    private State state;
    private StateMachine machine;

    private String oldName;
    private String newName;

    private Hashtable<String, Integer> oldActions = new Hashtable<String, Integer>();
    private Hashtable<String, Integer> newActions = new Hashtable<String, Integer>();

    public StatePropertyChanged(State state, StateMachine machine) {
        this.state = state;
        this.machine = machine;
        oldName = state.getName();
        for (Action a : state.getActions()) {
            oldActions.put(a.getActuatorName(), a.getValue());
        }
    }

    public boolean operationComplete() {
        newName = state.getName();

        boolean change = !newName.equals(oldName);

        for (Action a : state.getActions()) {
            newActions.put(a.getActuatorName(), a.getValue());

            if(!(oldActions.containsKey(a.getActuatorName()) && oldActions.get(a.getActuatorName()).equals(a.getValue()))){
                change=true;
            }
        }
        return change;
    }

    @Override
    public void undo() {
        state.setName(oldName);
        state.getActions().clear();
        for (Map.Entry<String, Integer> entry : oldActions.entrySet()) {
            state.addAction(new Action(entry.getKey(), entry.getValue()));
        }
        machine.fixTransitionEnds(state);
    }

    @Override
    public void redo() {
        state.setName(newName);
        state.getActions().clear();
        for (Map.Entry<String, Integer> entry : newActions.entrySet()) {
            state.addAction(new Action(entry.getKey(), entry.getValue()));
        }
        machine.fixTransitionEnds(state);
    }
}
