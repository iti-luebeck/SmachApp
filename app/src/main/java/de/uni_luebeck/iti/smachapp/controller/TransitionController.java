package de.uni_luebeck.iti.smachapp.controller;

import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.RectF;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.widget.Toast;

import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.BezierPath;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;
import de.uni_luebeck.iti.smachapp.utils.RectUtils;

/**
 * Created by Morten Mey on 28.04.2014.
 */
public class TransitionController implements ExtendedGestureListener {

    public static final float MARGIN = 25f;

    private StateMachineEditorController cont;

    private Path tempPath = new Path();
    private List<PointF> points = new LinkedList<PointF>();

    private boolean isLongPress = false;
    private boolean isScroll = false;

    private State begin = null;
    private Transition selected = null;
    private int closestPoint = 0;
    private PointF originalPoint = new PointF();
    private PointF lastPoint = new PointF();

    public TransitionController(StateMachineEditorController cont) {
        this.cont = cont;
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        begin = null;
        selected = null;
        tempPath.reset();
        points.clear();
        isLongPress = false;
        isScroll = false;

        PointF point = new PointF(motionEvent.getX(), motionEvent.getY());
        cont.getView().translatePoint(point);

        RectF rect = new RectF();
        for (State s : cont.getModel().getStateMachine()) {
            cont.getView().getStateRect(s, rect);
            if (rect.contains(point.x, point.y)) {
                begin = s;
                tempPath.moveTo(point.x, point.y);
                points.add(new PointF(point.x, point.y));
                cont.getView().setTempPath(tempPath);
                return true;
            }
        }

        float minDistance=BezierPath.MIN_DISTANCE;
        for (Transition trans : cont.getModel().getStateMachine().getTransitions()) {
            List<PointF> transPoints = trans.getPath().getPoints();
            for (int i = 0; i < transPoints.size() - 1; i++) {
                PointF curr = transPoints.get(i);
                float dist = PointUtils.distance(curr, point);

                if (dist < minDistance) {
                    minDistance = dist;
                    selected = trans;
                    lastPoint.set(point);
                    originalPoint.set(curr);

                    closestPoint = i;

                    if (i + 1 < transPoints.size()) {
                        PointF nextPoint = transPoints.get(i + 1);

                        float secondDist = PointUtils.distance(nextPoint, point);
                        if (dist > secondDist) {
                            closestPoint = i + 1;
                            originalPoint.set(nextPoint);
                            minDistance = secondDist;
                        }
                    }

                    break;
                }
            }
        }

         return selected!=null;
    }

    @Override
    public void onUp(MotionEvent e) {
        cont.getView().setTempPath(null);
        if (isLongPress) {
            return;
        }
        cont.getView().highlighteTransition(null);

        if (begin != null && isScroll) {
            State end = null;
            PointF point = new PointF(e.getX(), e.getY());
            cont.getView().translatePoint(point);
            RectF rect = new RectF();
            for (State s : cont.getModel().getStateMachine()) {
                cont.getView().getStateRect(s, rect);
                RectUtils.extendRect(rect, MARGIN);
                if (rect.contains(point.x, point.y)) {
                    end = s;
                    points.add(new PointF(point.x, point.y));
                    break;
                }
            }
            if (end == null) {
                Toast toast = Toast.makeText(cont.getView().getContext(), R.string.must_end_at_state, Toast.LENGTH_LONG);
                toast.show();
            } else if (begin == end) {
                Toast toast = Toast.makeText(cont.getView().getContext(), R.string.begin_must_not_be_end, Toast.LENGTH_LONG);
                toast.show();
            } else {
                cont.getView().getStateRect(begin, rect);
                BezierPath.removePointsInOval(rect, points);
                BezierPath.moveOnOval(rect, points.get(0));
                cont.getView().getStateRect(end, rect);
                BezierPath.removePointsInOval(rect, points);
                BezierPath.moveOnOval(rect, points.get(points.size() - 1));

                BezierPath.filterPoints(points);

                Transition newTrans = new Transition(begin, end, cont.getModel().getNextTransitionName(), new BezierPath(points));
                cont.getModel().getStateMachine().addTransition(newTrans);
            }
        } else if (selected != null && isScroll) {

            boolean connected = false;
            RectF rect = new RectF();

            if (closestPoint == 0) {
                for (State s : cont.getModel().getStateMachine()) {
                    cont.getView().getStateRect(s, rect);
                    RectUtils.extendRect(rect, MARGIN);
                    PointF p = selected.getPath().getPoints().get(0);
                    if (rect.contains(p.x, p.y)) {
                        connected = true;
                        if(selected.getPreviousState()!=null) {
                            selected.getPreviousState().removeTransition(selected);
                        }
                        selected.setPreviousState(s);
                        s.addTransition(selected);
                        RectUtils.extendRect(rect,-MARGIN);
                        BezierPath.moveOnOval(rect, p);
                        break;
                    }
                }
                if(!connected){
                    selected.getPath().getPoints().get(0).set(originalPoint);
                    Toast toast=Toast.makeText(cont.getView().getContext(),R.string.unconnected_transition,Toast.LENGTH_LONG);
                    toast.show();
                }

            } else if (closestPoint == selected.getPath().getPoints().size() - 1) {
                for (State s : cont.getModel().getStateMachine()) {
                    cont.getView().getStateRect(s, rect);
                    RectUtils.extendRect(rect, MARGIN);
                    PointF p = selected.getPath().getPoints().get(selected.getPath().getPoints().size() - 1);
                    if (rect.contains(p.x, p.y)) {
                        connected = true;
                        selected.setFollowerState(s);
                        RectUtils.extendRect(rect,-MARGIN);
                        BezierPath.moveOnOval(rect, p);
                        break;
                    }
                }
                if(!connected){
                    selected.getPath().getPoints().get(selected.getPath().getPoints().size()-1).set(originalPoint);
                    Toast toast=Toast.makeText(cont.getView().getContext(),R.string.unconnected_transition,Toast.LENGTH_LONG);
                    toast.show();
                }
            }

            selected.getPath().fixBeginning();
            selected.getPath().fixEnd();
        }
    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {
        cont.getView().highlighteTransition(selected);
    }

    @Override
    public boolean onSingleTapUp(MotionEvent motionEvent) {
        if (selected != null) {
            cont.showTransitionProperties(selected);
            return true;
        }
        return false;
    }

    @Override
    public boolean onScroll(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {

        isScroll = true;
        PointF point = new PointF(motionEvent2.getX(), motionEvent2.getY());
        cont.getView().translatePoint(point);

        if (begin != null) {
            tempPath.lineTo(point.x, point.y);
            points.add(new PointF(point.x, point.y));
            cont.getView().postInvalidate();
            return true;
        } else if (selected != null) {

            PointF temp = PointUtils.calculateDirection(lastPoint, point);
            lastPoint.set(point);

            selected.getPath().moveKnots(temp.x, temp.y, closestPoint);
            cont.getView().postInvalidate();

            return true;
        } else {
            return false;
        }
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {
        if (selected != null) {
            isLongPress = true;
            cont.showContextMenu();
        }
    }

    @Override
    public boolean onFling(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        return false;
    }

    @Override
    public boolean onContextItemSelected(MenuItem item) {
        cont.getView().highlighteTransition(null);
        switch (item.getItemId()) {
            case R.id.context_menu_delete:
                cont.getModel().getStateMachine().removeTransition(selected);
                return true;

            case R.id.context_menu_reset:
                selected.getPath().reset();
                return true;

            case R.id.context_menu_properties:
                cont.showTransitionProperties(selected);
                return true;

            default:
                return false;
        }
    }

}
