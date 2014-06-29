package de.uni_luebeck.iti.smachapp.app;

import android.app.Activity;
import android.os.Bundle;
import android.preference.PreferenceFragment;

/**
 * Created by Morten Mey on 28.06.2014.
 */
public class SettingsActivity extends Activity {

    @Override
    protected void onCreate(Bundle bundle) {
        super.onCreate(bundle);

        // Display the fragment as the main content.
        getFragmentManager().beginTransaction()
                .replace(android.R.id.content, new PreferenceFragment() {
                    @Override
                    public void onCreate(Bundle savedInstanceState) {
                        super.onCreate(savedInstanceState);
                        addPreferencesFromResource(R.xml.preferences);
                    }
                })
                .commit();

    }
}
