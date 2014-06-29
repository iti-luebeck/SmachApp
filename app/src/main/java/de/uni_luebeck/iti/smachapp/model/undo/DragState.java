package de.uni_luebeck.iti.smachapp.model.undo;

import android.graphics.PointF;

import java.util.Hashtable;
import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class DragState implements UndoableAction {

    private State state;
    private List<Transition> transitions;

    private float xBefore, yBefore;
    private float xAfter, yAfter;
    private Hashtable<Transition, List<PointF>> before = new Hashtable<Transition, List<PointF>>();
    private Hashtable<Transition, List<PointF>> after = new Hashtable<Transition, List<PointF>>();


    public DragState(State state, List<Transition> incomingTransitions) {
        this.state = state;
        this.transitions = new LinkedList<Transition>(incomingTransitions);
        transitions.addAll(state.getTransitions());
        xBefore = state.getX();
        yBefore = state.getY();

        for (Transition t : transitions) {
            before.put(t, PointUtils.copyPoints(t.getPath().getPoints()));
        }
    }

    public void operationDone() {
        xAfter = state.getX();
        yAfter = state.getY();

        for (Transition t : transitions) {
            after.put(t, PointUtils.copyPoints(t.getPath().getPoints()));
        }
    }

    @Override
    public void undo() {
        state.setX(xBefore);
        state.setY(yBefore);

        for (Transition t : transitions) {
            t.getPath().getPoints().clear();
            t.getPath().getPoints().addAll(PointUtils.copyPoints(before.get(t)));
            t.getPath().resize();
        }
    }

    @Override
    public void redo() {
        state.setX(xAfter);
        state.setY(yAfter);

        for (Transition t : transitions) {
            t.getPath().getPoints().clear();
            t.getPath().getPoints().addAll(PointUtils.copyPoints(after.get(t)));
            t.getPath().resize();
        }
    }
}
