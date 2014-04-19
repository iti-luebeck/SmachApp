package de.uni_luebeck.iti.smachapp.app;

import android.app.ActionBar;
import android.app.Activity;
import android.os.Bundle;

public class StateMachineEditor extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_state_machine_editor);
    }

    @Override
    protected void onResume(){
        super.onResume();
        ActionBar bar=getActionBar();
        bar.hide();
    }
}
