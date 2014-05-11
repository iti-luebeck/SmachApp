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

/**
 * Created by Morten Mey on 28.04.2014.
 */
public class TransitionController implements ExtendedGestureListener{

    private StateMachineEditorController cont;

    private RectF rect=new RectF();
    private PointF point=new PointF();
    private Path tempPath=new Path();
    private List<PointF> points=new LinkedList<PointF>();

    private State begin=null;

    public TransitionController(StateMachineEditorController cont){
        this.cont=cont;
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        begin=null;
        tempPath.reset();
        points.clear();
        for(State s:cont.getModel().getStateMachine()){
            cont.getView().getStateRect(s,rect);
            point.x=motionEvent.getX();
            point.y=motionEvent.getY();
            cont.getView().translatePoint(point);
            if(rect.contains(point.x,point.y)){
                begin=s;
                tempPath.moveTo(point.x,point.y);
                points.add(new PointF(point.x,point.y));
                cont.getView().setTempPath(tempPath);
                break;
            }
        }
        return begin!=null;
    }

    @Override
    public void onUp(MotionEvent e) {
        if(begin==null){
            return;
        }
        State end=null;
        for(State s:cont.getModel().getStateMachine()){
            cont.getView().getStateRect(s,rect);
            point.x=e.getX();
            point.y=e.getY();
            cont.getView().translatePoint(point);
            if(rect.contains(point.x,point.y)){
                end=s;
                points.add(new PointF(point.x,point.y));
                break;
            }
        }
        if(end==null){
            Toast toast= Toast.makeText(cont.getView().getContext(),R.string.must_end_at_state,Toast.LENGTH_LONG);
            toast.show();
        }else{
            BezierPath.filterPoints(points);
            Transition newTrans=new Transition(begin,end,cont.getModel().getNextTransitionName(),new BezierPath(points));
            cont.getModel().getStateMachine().addTransition(newTrans);
        }
        cont.getView().setTempPath(null);
    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {

    }

    @Override
    public boolean onSingleTapUp(MotionEvent motionEvent) {
        return false;
    }

    @Override
    public boolean onScroll(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        if(begin!=null){
            point.x=motionEvent2.getX();
            point.y=motionEvent2.getY();
            cont.getView().translatePoint(point);
            tempPath.lineTo(point.x,point.y);
            points.add(new PointF(point.x,point.y));
            cont.getView().postInvalidate();
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {

    }

    @Override
    public boolean onFling(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        return false;
    }

    @Override
    public boolean onContextItemSelected(MenuItem item) {
        return false;
    }

}
