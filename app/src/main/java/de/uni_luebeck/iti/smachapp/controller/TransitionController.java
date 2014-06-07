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

/**
 * Created by Morten Mey on 28.04.2014.
 */
public class TransitionController implements ExtendedGestureListener{

    private StateMachineEditorController cont;

    private RectF rect=new RectF();
    private PointF point=new PointF();
    private Path tempPath=new Path();
    private List<PointF> points=new LinkedList<PointF>();

    private boolean isLongPress=false;

    private State begin=null;
    private Transition selected=null;
    private int closestPoint=0;
    private PointF originalPoint=new PointF();
    private PointF lastPoint=new PointF();

    public TransitionController(StateMachineEditorController cont){
        this.cont=cont;
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        begin=null;
        selected=null;
        tempPath.reset();
        points.clear();
        isLongPress=false;

        point.set(motionEvent.getX(),motionEvent.getY());
        cont.getView().translatePoint(point);

        for(State s:cont.getModel().getStateMachine()){
            cont.getView().getStateRect(s,rect);
            if(rect.contains(point.x,point.y)){
                begin=s;
                tempPath.moveTo(point.x,point.y);
                points.add(new PointF(point.x,point.y));
                cont.getView().setTempPath(tempPath);
                break;
            }
        }

        for(Transition trans:cont.getModel().getStateMachine().getTransitions()){
            List<PointF> transPoints=trans.getPath().getPoints();
            for(int i=1;i<transPoints.size()-1;i++){
                PointF curr=transPoints.get(i);
                float dist=PointUtils.distance(curr,point);

                if(dist<BezierPath.RELAXED_MIN_DISTANCE){
                    selected=trans;
                    lastPoint.set(point);
                    originalPoint.set(point);

                    closestPoint=i;

                    if(i+2<transPoints.size()){
                        PointF nextPoint=transPoints.get(i+1);

                        if(dist>PointUtils.distance(nextPoint,point)){
                            closestPoint=i+1;
                        }
                    }

                    break;
                }
            }
            if(selected!=null){
                break;
            }
        }

        return begin!=null || selected!=null;
    }

    @Override
    public void onUp(MotionEvent e) {
        cont.getView().setTempPath(null);
        if(isLongPress){
            return;
        }
        cont.getView().highlighteTransition(null);

        if(begin!=null) {
            State end = null;
            point.set(e.getX(), e.getY());
            cont.getView().translatePoint(point);
            for (State s : cont.getModel().getStateMachine()) {
                cont.getView().getStateRect(s, rect);
                if (rect.contains(point.x, point.y)) {
                    end = s;
                    points.add(new PointF(point.x, point.y));
                    break;
                }
            }
            if (end == null) {
                Toast toast = Toast.makeText(cont.getView().getContext(), R.string.must_end_at_state, Toast.LENGTH_LONG);
                toast.show();
            }else if(begin==end){
                Toast toast=Toast.makeText(cont.getView().getContext(),R.string.begin_must_not_be_end,Toast.LENGTH_LONG);
                toast.show();
            } else {
                RectF rect = new RectF();
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
        }else if(selected!=null){
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
        if(selected!=null){
            cont.showTransitionProperties(selected);
            return true;
        }
        return false;
    }

    @Override
    public boolean onScroll(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {

        point.x=motionEvent2.getX();
        point.y=motionEvent2.getY();
        cont.getView().translatePoint(point);

        if(begin!=null){
            tempPath.lineTo(point.x,point.y);
            points.add(new PointF(point.x,point.y));
            cont.getView().postInvalidate();
            return true;
        }else if(selected!=null){

            PointF temp=PointUtils.calculateDirection(lastPoint, point);
            lastPoint.set(point);

            selected.getPath().moveKnots(temp.x,temp.y,closestPoint);
            cont.getView().postInvalidate();

            return true;
        }else{
            return false;
        }
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {
        if(selected!=null){
            isLongPress=true;
            cont.showContextMenu();
        }
    }

    @Override
    public boolean onFling(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        return false;
    }

    @Override
    public boolean onContextItemSelected(MenuItem item) {
        switch(item.getItemId()){
            case R.id.context_menu_delete:
                cont.getModel().getStateMachine().removeTransition(selected);
                cont.getView().highlighteTransition(null);
                return true;

            case R.id.context_menu_properties:
                cont.showTransitionProperties(selected);
                return true;

            default:
                return false;
        }
    }

}
