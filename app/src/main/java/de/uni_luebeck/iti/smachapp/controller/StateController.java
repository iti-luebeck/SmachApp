package de.uni_luebeck.iti.smachapp.controller;

import android.graphics.PointF;
import android.graphics.RectF;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.widget.Toast;

import java.util.List;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.BezierPath;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;

/**
 * A controller for handling input, when the user is in state editing mode.
 * Created by Morten Mey on 28.04.2014.
 */
public class StateController implements ExtendedGestureListener{

    private StateMachineEditorController cont;

    private State dragged=null;
    private PointF originalPoint=new PointF();
    private PointF lastPoint=new PointF();
    private List<State> incomingTransitions;
    private double threshold=0;

    private RectF rect=new RectF();
    private PointF point=new PointF();
    private PointF temp=new PointF();

    private boolean isLongPress=false;

    public StateController(StateMachineEditorController cont){
        this.cont=cont;
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        dragged=null;
        for(State s:cont.getModel().getStateMachine()){
            cont.getView().getStateRect(s,rect);
            point.set(motionEvent.getX(),motionEvent.getY());
            cont.getView().translatePoint(point);
            if(rect.contains(point.x,point.y)){
                dragged=s;
                originalPoint.set(dragged.getX(),dragged.getY());
                lastPoint.set(originalPoint);
                incomingTransitions=cont.getModel().getStateMachine().getPreviousStates(dragged);
                threshold=0;
                break;
            }
        }
        return dragged!=null;
    }

    @Override
    public void onUp(MotionEvent e) {
        if(!isLongPress) {
            dragged = null;
            cont.getView().highlighteState(null);
        }
        isLongPress=false;
    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {
        cont.getView().highlighteState(dragged);
    }

    @Override
    public boolean onSingleTapUp(MotionEvent motionEvent) {
        if(dragged!=null){
            return false;
        }

        point.x=motionEvent.getX();
        point.y=motionEvent.getY();
        cont.getView().translatePoint(point);
        State s=new State(cont.getModel().getNextStateName(),point.x,point.y);
        cont.getModel().getStateMachine().addState(s);
        cont.getView().postInvalidate();

        return true;
    }

    @Override
    public boolean onScroll(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        if(dragged!=null){
            point.x=motionEvent2.getX();
            point.y=motionEvent2.getY();
            cont.getView().translatePoint(point);

            dragged.setCenter(point.x,point.y);

            temp.x=point.x-originalPoint.x;
            temp.y=point.y-originalPoint.y;
            threshold=Math.max(temp.length()/ BezierPath.MIN_DISTANCE,threshold);
            temp.x=point.x-lastPoint.x;
            temp.y=point.y-lastPoint.y;
            lastPoint.set(point);

            for(Transition trans:dragged){
                List<PointF> points=trans.getPath().getPoints();
                for(int i=0;i<=threshold&&i<points.size()-1;i++){
                    PointF curr=points.get(i);
                    curr.x+=temp.x;
                    curr.y+=temp.y;
                }
                trans.getPath().updateCurveControlPoints();
            }

            for(State prev:incomingTransitions){
                for(Transition trans:prev){
                    if(trans.getFollowerState()!=dragged){
                        continue;
                    }

                    List<PointF> points=trans.getPath().getPoints();
                    for(int i=0;i<=threshold&&i<points.size()-1;i++){
                        PointF curr=points.get(points.size()-1-i);
                        curr.x+=temp.x;
                        curr.y+=temp.y;
                    }
                    trans.getPath().updateCurveControlPoints();
                }
            }

            cont.getView().postInvalidate();
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {
        if(dragged!=null) {
            cont.showContextMenu();
            isLongPress = true;
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
                if(dragged.isInitialState()){
                    Toast toast= Toast.makeText(cont.getView().getContext(),R.string.cant_delete_initial_state,Toast.LENGTH_LONG);
                    toast.show();
                }else {
                    cont.getModel().getStateMachine().removeState(dragged);
                }
                dragged=null;
                cont.getView().highlighteState(null);
                return true;

            case R.id.context_menu_properties:
                return true;

            default:
                return false;
        }
    }
}
