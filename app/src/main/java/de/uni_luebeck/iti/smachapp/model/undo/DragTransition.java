package de.uni_luebeck.iti.smachapp.model.undo;

import android.graphics.PointF;

import java.util.List;

import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class DragTransition implements UndoableAction {

    private Transition transition;

    private State prevBefore;
    private State nextBefore;

    private State prefAfter;
    private State nextAfter;

    private List<PointF> before;
    private List<PointF> after;

    public DragTransition(Transition transition) {
        this.transition = transition;
        prevBefore = transition.getPreviousState();
        nextBefore = transition.getFollowerState();
        before = PointUtils.copyPoints(transition.getPath().getPoints());
    }


    public void operationDone() {
        prefAfter = transition.getPreviousState();
        nextAfter = transition.getFollowerState();
        after = PointUtils.copyPoints(transition.getPath().getPoints());
    }

    @Override
    public void undo() {
        transition.setFollowerState(nextBefore);
        transition.setPreviousState(prevBefore);
        transition.getPath().getPoints().clear();
        transition.getPath().getPoints().addAll(PointUtils.copyPoints(before));
        transition.getPath().resize();
    }

    @Override
    public void redo() {
        transition.setPreviousState(prefAfter);
        transition.setFollowerState(nextAfter);
        transition.getPath().getPoints().clear();
        transition.getPath().getPoints().addAll(PointUtils.copyPoints(after));
        transition.getPath().resize();
    }
}
