package de.uni_luebeck.iti.smachapp.controller;

import android.os.AsyncTask;
import android.view.ActionMode;
import android.view.GestureDetector;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import android.widget.Toast;

import java.io.IOException;

import de.uni_luebeck.iti.smachGenerator.SmachAutomat;
import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.app.StateMachineEditor;
import de.uni_luebeck.iti.smachapp.model.BeepRobot;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.model.XMLSaverLoader;
import de.uni_luebeck.iti.smachapp.view.StateMachineView;

/**
 * Created by Morten Mey on 28.04.2014.
 */
public class StateMachineEditorController implements View.OnTouchListener, ScaleGestureDetector.OnScaleGestureListener, GestureDetector.OnGestureListener {

    private EditorModel model;
    private StateMachineView view;
    private StateMachineEditor activity;
    private ExtendedGestureListener subController;

    private GestureDetector detector;
    private ScaleGestureDetector scaleDetector;

    private boolean isMulitTouch = false;
    private ActionMode actionMode = null;

    public StateMachineEditorController(EditorModel model, StateMachineView view, StateMachineEditor activity) {
        this.model = model;
        this.view = view;
        this.activity = activity;
        view.setModel(model);
        view.setOnTouchListener(this);
        detector = new GestureDetector(view.getContext(), this);
        scaleDetector = new ScaleGestureDetector(view.getContext(), this);

        switch (model.getCurrentState()) {
            case EDIT_STATES:
                subController = new StateController(this);
                break;
            case EDIT_TRANSITIONS:
                subController = new TransitionController(this);
                break;
        }
    }

    public EditorModel getModel() {
        return model;
    }

    public StateMachineView getView() {
        return view;
    }

    public void modeSwitch(EditorModel.EditorState state) {
        model.setCurrentState(state);

        switch (state) {
            case EDIT_STATES:
                subController = new StateController(this);
                break;
            case EDIT_TRANSITIONS:
                subController = new TransitionController(this);
                break;
        }
    }

    public void resetView() {
        view.reset();
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        subController.onDown(motionEvent);
        return true;
    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {
        subController.onShowPress(motionEvent);
    }

    @Override
    public boolean onSingleTapUp(MotionEvent motionEvent) {
        return subController.onSingleTapUp(motionEvent);
    }

    @Override
    public boolean onScroll(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        boolean handled = false;

        if (!isMulitTouch) {
            handled = subController.onScroll(motionEvent, motionEvent2, v, v2);
        }

        if (!handled) {
            view.translate(v, v2);
        }

        return true;
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {
        subController.onLongPress(motionEvent);
    }

    @Override
    public boolean onFling(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        return false;
    }

    @Override
    public boolean onScale(ScaleGestureDetector scaleGestureDetector) {
        view.scale(scaleGestureDetector.getScaleFactor());
        return true;
    }

    @Override
    public boolean onScaleBegin(ScaleGestureDetector scaleGestureDetector) {
        return true;
    }

    @Override
    public void onScaleEnd(ScaleGestureDetector scaleGestureDetector) {

    }

    public void showContextMenu() {
        actionMode = activity.startActionMode(new ActionMode.Callback() {
            @Override
            public boolean onCreateActionMode(ActionMode mode, Menu menu) {
                MenuInflater inflater = mode.getMenuInflater();
                inflater.inflate(R.menu.action_mode, menu);

                switch (model.getCurrentState()) {
                    case EDIT_STATES:
                        mode.getMenu().findItem(R.id.context_menu_reset).setVisible(false);
                        break;

                    case EDIT_TRANSITIONS:
                        mode.getMenu().findItem(R.id.context_menu_make_initial).setVisible(false);
                        break;
                }

                return true;

            }

            @Override
            public boolean onPrepareActionMode(ActionMode mode, Menu menu) {
                return false;
            }

            @Override
            public boolean onActionItemClicked(ActionMode mode, MenuItem menuItem) {
                boolean res = subController.onContextItemSelected(menuItem);
                actionMode.finish();
                return res;
            }

            @Override
            public void onDestroyActionMode(ActionMode mode) {
                subController.actionModeFinished();
                actionMode = null;
            }
        });
    }

    public boolean updateSelection(int selectionSize) {
        if (actionMode == null) {
            return false;
        }


        if (selectionSize == 0) {
            actionMode.finish();
            return false;
        } else if (selectionSize > 1) {
            actionMode.getMenu().findItem(R.id.context_menu_make_initial).setVisible(false);
            actionMode.getMenu().findItem(R.id.context_menu_properties).setVisible(false);
            return true;

        } else {
            switch (model.getCurrentState()) {
                case EDIT_STATES:
                    actionMode.getMenu().findItem(R.id.context_menu_make_initial).setVisible(true);
                    //falls through
                case EDIT_TRANSITIONS:
                    actionMode.getMenu().findItem(R.id.context_menu_properties).setVisible(true);
                    break;
            }
            return true;
        }
    }

    @Override
    public boolean onTouch(View view, MotionEvent e) {
        switch (e.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                isMulitTouch = false;
                break;
            case MotionEvent.ACTION_POINTER_DOWN:
                isMulitTouch = true;
                break;

            case MotionEvent.ACTION_UP:
                subController.onUp(e);
        }

        scaleDetector.onTouchEvent(e);
        detector.onTouchEvent(e);
        return true;
    }

    public void showStateProperties(State s) {
        activity.showStateProperties(s);
    }

    public void showTransitionProperties(Transition t) {
        activity.showTransitionProperites(t);
    }

    public void play(String address) {
        BeepRobot robot = model.getRobot();
        XMLSaverLoader.PATH.mkdirs();
        SmachAutomat automat = new SmachAutomat(model.getStateMachine().getStates(), model.getRobot().getSensors(), model.getRobot().getActuators(), "zusmoro_state_machine");
        automat.saveToFile(model.getPythonFile());

        Toast toast = Toast.makeText(activity, R.string.connecting, Toast.LENGTH_LONG);
        toast.show();
        new NetworkTask(robot, address).execute((Void) null);

    }

    public void resumed() {
        subController.resumed();
    }

    public void save() {
        try {
            XMLSaverLoader.save(model);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    private class NetworkTask extends AsyncTask<Void, Void, Void> {

        private BeepRobot robot;
        private String address;

        private NetworkTask(BeepRobot ro, String add) {
            robot = ro;
            address = add;
        }

        @Override
        protected Void doInBackground(Void... voids) {
            try {

                robot.connect(address);
                robot.transmit(model.getPythonFile());
                robot.play();
            } catch (Exception ex) {
                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast toast = Toast.makeText(activity, R.string.connnectionFailure, Toast.LENGTH_LONG);
                        toast.show();
                    }
                });
            }
            return null;
        }
    }
}
