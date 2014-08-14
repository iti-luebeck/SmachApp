package de.uni_luebeck.iti.smachapp.controller;

import android.os.AsyncTask;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import android.widget.Toast;

import de.uni_luebeck.iti.smachapp.app.DebugActivity;
import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.BeepRobot;
import de.uni_luebeck.iti.smachapp.model.DebugModel;
import de.uni_luebeck.iti.smachapp.model.RosNode;
import de.uni_luebeck.iti.smachapp.view.StateMachineView;

/**
 * Created by Morten Mey on 04.07.2014.
 */
public class DebugController implements GestureDetector.OnGestureListener, ScaleGestureDetector.OnScaleGestureListener, View.OnTouchListener {

    private static int DIST_FROM_EDGE=10;

    private DebugModel model;
    private DebugActivity activity;
    private StateMachineView view;

    private GestureDetector detector;
    private ScaleGestureDetector scaleDetector;
    private boolean switched=false;

    private RosNode node;

    public DebugController(DebugModel model, DebugActivity activity) {
        this.model = model;
        this.activity = activity;

        detector = new GestureDetector(activity, this);
        scaleDetector = new ScaleGestureDetector(activity, this);
    }

    public void setView(StateMachineView view) {
        this.view = view;
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        switched=false;
        return true;
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

        if(switched){
            return true;
        }

        int width=view.getWidth();

        if(motionEvent.getX()<DIST_FROM_EDGE|| motionEvent.getX()>width-DIST_FROM_EDGE){
            activity.setNavIndex((activity.getNavIndex()+1)%2);
            switched=true;
            return true;
        }

        int nav = activity.getNavIndex();

        if (nav == 0) {

            view.translate(v, v2);
            return true;
        }

        return false;
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {

    }

    @Override
    public boolean onFling(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        return false;
    }

    @Override
    public boolean onScale(ScaleGestureDetector scaleGestureDetector) {
        int nav = activity.getNavIndex();
        if (nav == 0) {
            view.scale(scaleGestureDetector.getScaleFactor());
            return true;
        }
        return false;
    }

    @Override
    public boolean onScaleBegin(ScaleGestureDetector scaleGestureDetector) {
        return activity.getNavIndex() == 0;
    }

    @Override
    public void onScaleEnd(ScaleGestureDetector scaleGestureDetector) {

    }

    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {
        detector.onTouchEvent(motionEvent);
        scaleDetector.onTouchEvent(motionEvent);

        return true;
    }

    public void play() {
        Toast toast = Toast.makeText(activity, R.string.connecting, Toast.LENGTH_SHORT);
        toast.show();
        new StartTask(model.getEditor().getRobot(), model.getIp()).execute((Void)null);

    }

    public void stop(boolean disconnect){
        new StopTask(model.getEditor().getRobot()).execute(disconnect);
    }

    private class StartTask extends AsyncTask<Void, Void, Void> {

        private BeepRobot robot;
        private String address;

        private StartTask(BeepRobot ro, String add) {
            robot = ro;
            address = add;
        }

        @Override
        protected Void doInBackground(Void... connect) {
            try {
                if(!robot.isConnected()) {
                    robot.connect(address);
                    node=new RosNode(model,address);
                    node.startNode();
                    robot.transmit(model.getEditor().getPythonFile());
                }
                robot.play();
                model.setRunning(true);
                activity.invalidateOptionsMenu();
                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast toast = Toast.makeText(activity,R.string.connectionSuccess,Toast.LENGTH_SHORT);
                        toast.show();
                    }
                });
            } catch (Exception ex) {
                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast toast = Toast.makeText(activity, R.string.connnectionFailure, Toast.LENGTH_LONG);
                        toast.show();
                    }
                });
                ex.printStackTrace();
            }
            return null;
        }
    }

    private class StopTask extends AsyncTask<Boolean,Void,Void>{

        private BeepRobot robot;

        private StopTask(BeepRobot robot){
            this.robot=robot;
        }

        @Override
        protected Void doInBackground(Boolean... booleans) {
            try{
                robot.stop();
                model.setRunning(false);

                if(booleans[0]){
                    node.stopNode();
                    robot.disconnect();
                }

                activity.invalidateOptionsMenu();

            }catch (Exception ex){
                ex.printStackTrace();
            }

            return null;
        }
    }
}
