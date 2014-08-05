package de.uni_luebeck.iti.smachapp.app;

import android.app.ActionBar;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.FragmentTransaction;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.text.InputType;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;

import de.uni_luebeck.iti.smachapp.controller.StateMachineEditorController;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.model.XMLSaverLoader;
import de.uni_luebeck.iti.smachapp.model.undo.UndoListener;
import de.uni_luebeck.iti.smachapp.model.undo.UndoManager;
import de.uni_luebeck.iti.smachapp.view.StateMachineView;

public class StateMachineEditor extends Activity implements UndoListener {

    private static final String USE_OLD_MODEL = "USE_OLD_EDITOR_MODEL";
    private static final String TAB_INDEX = "TAB_INDEX";
    public static final String LAST_IP = "LAST_IP";
    public static final String IP = "IP";
    private static EditorModel oldModel = null;

    private StateMachineEditorController controller;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Intent intent = getIntent();

        EditorModel model = null;
        boolean restoreTabIndex = false;

        if (savedInstanceState != null && savedInstanceState.getBoolean(USE_OLD_MODEL, false) && oldModel != null) {
            model = oldModel;
            oldModel = null;
            restoreTabIndex = true;
        } else if (intent.getStringExtra("action").equals("new")) {
            model = new EditorModel(intent.getStringExtra("name"));
            oldModel = null;

            State state = new State(model.getNextStateName(), 0, 0, true);
            model.getStateMachine().addState(state);
        } else if (intent.getStringExtra("action").equals("load")) {
            File f = new File(intent.getData().getPath());
            try {
                model = XMLSaverLoader.load(f);
            } catch (Exception e) {
                Toast toast = Toast.makeText(this, R.string.failed_to_load, Toast.LENGTH_LONG);
                toast.show();
                e.printStackTrace();
                finish();
            }
        } else {
            throw new IllegalArgumentException();
        }

        model.getUndoManager().addListener(this);

        setContentView(R.layout.activity_state_machine_editor);

        StateMachineView view = (StateMachineView) findViewById(R.id.editorView);
        registerForContextMenu(view);

        controller = new StateMachineEditorController(model, view, this);

        ActionBar.TabListener listener = new ActionBar.TabListener() {
            @Override
            public void onTabSelected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
                switch (tab.getPosition()) {
                    case 0:
                        controller.modeSwitch(EditorModel.EditorState.EDIT_STATES);
                        break;
                    case 1:
                        controller.modeSwitch(EditorModel.EditorState.EDIT_TRANSITIONS);
                        break;
                }
            }

            @Override
            public void onTabUnselected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {

            }

            @Override
            public void onTabReselected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {

            }
        };

        ActionBar bar = getActionBar();
        bar.setNavigationMode(ActionBar.NAVIGATION_MODE_TABS);
        ActionBar.Tab tab = bar.newTab().setTabListener(listener).setText(R.string.edit_states);
        bar.addTab(tab);
        tab = bar.newTab().setTabListener(listener).setText(R.string.edit_transitions);
        bar.addTab(tab);

        if (restoreTabIndex) {
            bar.setSelectedNavigationItem(savedInstanceState.getInt(TAB_INDEX));
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu items for use in the action bar
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.editor_action_bar, menu);
        return super.onCreateOptionsMenu(menu);
    }

    @Override
    public boolean onPrepareOptionsMenu(Menu menu) {
        boolean isAvail = controller.getModel().getUndoManager().isUndoAvailable();
        MenuItem item = menu.findItem(R.id.action_undo);

        item.setEnabled(isAvail);
        if (isAvail) {
            item.getIcon().setAlpha(255);
        } else {
            item.getIcon().setAlpha(77);
        }

        isAvail = controller.getModel().getUndoManager().isRedoAvailable();
        item = menu.findItem(R.id.action_redo);

        item.setEnabled(isAvail);
        if (isAvail) {
            item.getIcon().setAlpha(255);
        } else {
            item.getIcon().setAlpha(77);
        }


        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle presses on the action bar items
        Intent intent;
        switch (item.getItemId()) {
            case R.id.action_play:
                transmit();
                break;
            case R.id.action_undo:
                controller.getModel().getUndoManager().undo();
                controller.getView().postInvalidate();
                break;
            case R.id.action_redo:
                controller.getModel().getUndoManager().redo();
                controller.getView().postInvalidate();
                break;
            case R.id.action_center:
                controller.resetView();
                break;
            case R.id.action_properties:
                MachineProperty.setupModel(controller.getModel());
                intent = new Intent(this, MachineProperty.class);
                startActivity(intent);
                break;
            case R.id.action_settings:
                intent = new Intent(this, SettingsActivity.class);
                startActivity(intent);
                break;
        }

        return true;
    }

    @Override
    protected void onResume() {
        super.onResume();
        getActionBar().setTitle(controller.getModel().getStateMachine().getName());
        findViewById(R.id.editorView).postInvalidate();
        controller.resumed();
    }

    @Override
    protected void onPause() {
        super.onPause();
        controller.save();
    }

    @Override
    protected void onSaveInstanceState(Bundle bundle) {
        super.onSaveInstanceState(bundle);
        oldModel = controller.getModel();
        bundle.putBoolean(USE_OLD_MODEL, true);
        bundle.putInt(TAB_INDEX, getActionBar().getSelectedNavigationIndex());
    }

    public void showStateProperties(State s) {
        Intent intent = new Intent(this, StateProperty.class);
        StateProperty.setupState(s, controller.getModel().getRobot(),controller.getModel().getStateMachine());
        startActivity(intent);
    }

    public void showTransitionProperites(Transition t) {
        Intent intent = new Intent(this, TransitionProperty.class);
        TransitionProperty.setupTransition(t, controller.getModel().getRobot(),controller.getModel().getStateMachine());
        startActivity(intent);
    }

    public void transmit() {
        controller.compile();

        final SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);

        AlertDialog.Builder builder = new AlertDialog.Builder(this);

        builder.setTitle(R.string.enterAddress);
        final EditText text = new EditText(this);
        text.setInputType(InputType.TYPE_CLASS_PHONE);
        text.setImeOptions(EditorInfo.IME_ACTION_SEND);
        text.setText(pref.getString(LAST_IP, ""));
        text.selectAll();
        builder.setView(text);

        final Intent intent = new Intent(this, DebugActivity.class);
        DebugActivity.setup(controller.getModel());
        final Activity activity = this;

        builder.setPositiveButton(R.string.send, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                String ip = text.getText().toString();
                SharedPreferences.Editor edit = pref.edit();
                edit.putString(LAST_IP, ip);
                edit.commit();
                intent.putExtra(IP, ip);
                activity.startActivity(intent);
            }
        });

        builder.setNegativeButton(R.string.disrecard, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
            }
        });

        final AlertDialog dia = builder.show();
        text.postDelayed(new Runnable() {
            @Override
            public void run() {
                InputMethodManager keyboard = (InputMethodManager)
                        getSystemService(Context.INPUT_METHOD_SERVICE);
                keyboard.showSoftInput(text, 0);
            }
        }, 200);

        text.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView textView, int i, KeyEvent keyEvent) {
                dia.dismiss();
                String ip = text.getText().toString();
                SharedPreferences.Editor edit = pref.edit();
                edit.putString(LAST_IP, ip);
                edit.commit();
                intent.putExtra(IP, ip);
                activity.startActivity(intent);
                return true;
            }
        });

    }

    @Override
    public void operationPerformed(UndoManager manager) {
        invalidateOptionsMenu();
    }
}
