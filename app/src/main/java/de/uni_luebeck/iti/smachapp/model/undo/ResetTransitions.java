package de.uni_luebeck.iti.smachapp.model.undo;

import android.graphics.PointF;

import java.util.ArrayList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.BezierPath;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class ResetTransitions implements UndoableAction {

    private List<Transition> transitions;
    private List<List<PointF>> oldPoints;

    public ResetTransitions(List<Transition> transitions) {
        this.transitions = new ArrayList<Transition>(transitions);
        oldPoints = new ArrayList<List<PointF>>(transitions.size());

        for (Transition t : transitions) {
            oldPoints.add(PointUtils.copyPoints(t.getPath().getPoints()));
        }
    }

    @Override
    public void undo() {
        for (int i = 0; i < transitions.size(); i++) {
            BezierPath path = transitions.get(i).getPath();
            path.getPoints().clear();
            path.getPoints().addAll(oldPoints.get(i));
            path.resize();
        }
    }

    @Override
    public void redo() {
        for (Transition t : transitions) {
            t.getPath().reset();
        }
    }
}
