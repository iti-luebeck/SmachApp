package de.uni_luebeck.iti.smachapp.app;

import android.app.Activity;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.Preference;
import android.preference.PreferenceFragment;
import android.preference.PreferenceManager;
import android.widget.Toast;

/**
 * Created by Morten Mey on 28.06.2014.
 */
public class SettingsActivity extends Activity {

    @Override
    protected void onCreate(Bundle bundle) {
        super.onCreate(bundle);

        final Activity activity=this;

        // Display the fragment as the main content.
        getFragmentManager().beginTransaction()
                .replace(android.R.id.content, new PreferenceFragment() {
                    @Override
                    public void onCreate(Bundle savedInstanceState) {
                        super.onCreate(savedInstanceState);
                        addPreferencesFromResource(R.xml.preferences);

                        Preference button = (Preference)findPreference("button");
                        button.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
                            @Override
                            public boolean onPreferenceClick(Preference arg0) {
                                SharedPreferences pref= PreferenceManager.getDefaultSharedPreferences(activity);
                                pref.edit();
                                SharedPreferences.Editor edit=pref.edit();
                                edit.putString(StateMachineEditor.LAST_IP,"");
                                edit.commit();
                                Toast toast=Toast.makeText(activity,R.string.ip_deleted,Toast.LENGTH_LONG);
                                toast.show();
                                return true;
                            }
                        });
                    }
                })
                .commit();

    }
}
