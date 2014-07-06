package de.uni_luebeck.iti.smachapp.app;

import android.app.ActionBar;
import android.app.Activity;
import android.app.FragmentTransaction;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import java.util.ArrayList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.controller.DebugController;
import de.uni_luebeck.iti.smachapp.model.DebugModel;
import de.uni_luebeck.iti.smachapp.model.DebugModelObserver;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.view.SensorView;
import de.uni_luebeck.iti.smachapp.view.StateMachineView;

/**
 * Created by Morten Mey on 04.07.2014.
 */
public class DebugActivity extends Activity implements DebugModelObserver{

    private static String TAB_INDEX = "TAB_INDEX";

    private StateMachineView smView;
    private SensorView sensorView;

    private DebugController controller;

    private DebugModel model;
    private static EditorModel setupModel;

    private ViewGroup group;

    private List<State> currentState=new ArrayList<State>(1);
    private List<Transition> lastTransition=new ArrayList<Transition>(1);

    public static void setup(EditorModel model) {
        setupModel = model;
    }

    @Override
    protected void onCreate(Bundle bundle) {
        super.onCreate(bundle);

        this.model = new DebugModel(setupModel);
        model.addObserver(this);

        controller = new DebugController(model, this);

        group=new LinearLayout(this);
        group.setOnTouchListener(controller);

        smView=new StateMachineView(this);
        sensorView=new SensorView(this);
        smView.setModel(model.getEditor());
        currentState.add(model.getCurrentState());
        smView.highlighteTransitions(lastTransition);
        smView.highlighteStates(currentState);
        sensorView.setModel(model);

        controller.setView(smView);

        final Activity activity = this;

        ActionBar.TabListener listener = new ActionBar.TabListener() {
            @Override
            public void onTabSelected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
                switch (tab.getPosition()) {
                    case 0:
                        group.addView(smView);
                        break;
                    case 1:
                        group.addView(sensorView);
                        break;
                }
                activity.invalidateOptionsMenu();
            }

            @Override
            public void onTabUnselected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
                switch (tab.getPosition()) {
                    case 0:
                        group.removeAllViews();
                        break;
                    case 1:
                        group.removeAllViews();
                        break;
                }
            }

            @Override
            public void onTabReselected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
            }
        };

        ActionBar bar = getActionBar();
        bar.setNavigationMode(ActionBar.NAVIGATION_MODE_TABS);
        ActionBar.Tab tab = bar.newTab().setTabListener(listener).setText(R.string.debug_automat);
        bar.addTab(tab);
        tab = bar.newTab().setTabListener(listener).setText(R.string.sensor_data);
        bar.addTab(tab);

        if (bundle != null) {
            bar.setSelectedNavigationItem(bundle.getInt(TAB_INDEX));
        }
        //controller.play(getIntent().getStringExtra("url"),true);

        setContentView(group);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu items for use in the action bar
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.debug_action_bar, menu);
        return super.onCreateOptionsMenu(menu);
    }

    @Override
    public boolean onPrepareOptionsMenu(Menu menu) {
        if (getNavIndex() == 1) {
            menu.findItem(R.id.action_center).setVisible(false);
        }
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.action_center:
                smView.reset();
                break;
        }
        return true;
    }

    @Override
    protected void onSaveInstanceState(Bundle bundle) {
        super.onSaveInstanceState(bundle);
        bundle.putInt(TAB_INDEX, getActionBar().getSelectedNavigationIndex());
    }

    public int getNavIndex() {
        return getActionBar().getSelectedNavigationIndex();
    }

    public void setNavIndex(int i) {
        getActionBar().setSelectedNavigationItem(i);
    }

    @Override
    public void onModelChange(DebugModel model) {
        currentState.clear();
        lastTransition.clear();
        currentState.add(model.getCurrentState());
        lastTransition.add(model.getLastTransition());
        smView.postInvalidate();
    }
}
