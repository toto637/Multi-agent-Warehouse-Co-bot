package com.gp.maw;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.Toast;

import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.Response;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.JsonObjectRequest;
import com.android.volley.toolbox.Volley;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

import HelperClasses.VolleySingleton;

public class DestActivity extends AppCompatActivity {

    String rid;

    public final List<String> locations = new ArrayList<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_dest);

        final RadioGroup radioGroup = findViewById(R.id.destRadioGroup);

        Intent intent = getIntent();
        rid = intent.getStringExtra("id");
        JSONObject jsonObject = new JSONObject();
        if (intent != null) {
            try {
                jsonObject.put("id", intent.getStringExtra("id"));
                jsonObject.put("uid", VolleySingleton.getInstance(getApplicationContext()).getUid());
            } catch (JSONException e) {
                e.printStackTrace();
            }
        }

        VolleySingleton volleySingleton = VolleySingleton.getInstance(getApplicationContext());
        RequestQueue requestQueue = volleySingleton.getRequestQueue();

        JsonObjectRequest jsonObjectRequest = new JsonObjectRequest(
                Request.Method.POST,
                MainActivity.api_address+"retrieve-valid-destinations",
                jsonObject,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        try {
                            JSONArray locJSONArray = response.getJSONArray("destinations");
                            for (int i = 0; i < locJSONArray.length(); i++) {
                                RadioButton radioButton = new RadioButton(getApplicationContext());
                                radioButton.setText(locJSONArray.getString(i));
                                locations.add(locJSONArray.getString(i));
                                radioButton.setId(i);  // Set a unique ID for each RadioButton
                                radioGroup.addView(radioButton);
                            }
                        } catch (JSONException e) {
                            Toast.makeText(getApplicationContext(), "Couldn't establish a connection with the server.", Toast.LENGTH_SHORT).show();
                        }
                    }
                }, new Response.ErrorListener() {
            @Override
            public void onErrorResponse(VolleyError error) {
                Toast.makeText(getApplicationContext(), "Couldn't establish a connection with the server.", Toast.LENGTH_SHORT).show();
            }
        }
        );
        requestQueue.add(jsonObjectRequest);
    }

    public void onSubmit(View view) {
        final RadioGroup radioGroup = findViewById(R.id.destRadioGroup);
        Intent intent = getIntent();

        if (radioGroup.getCheckedRadioButtonId() == -1) {
            Toast.makeText(getApplicationContext(), "Missing fields.", Toast.LENGTH_SHORT).show();
        } else {
            JSONObject jsonObject = new JSONObject();
            if (intent != null) {
                try {
                    jsonObject.put("uid", VolleySingleton.getInstance(getApplicationContext()).getUid());
                    jsonObject.put("destination", locations.get(radioGroup.getCheckedRadioButtonId()));
                } catch (JSONException e) {
                    e.printStackTrace();
                }
            }

            VolleySingleton volleySingleton = VolleySingleton.getInstance(getApplicationContext());
            RequestQueue requestQueue = volleySingleton.getRequestQueue();

            JsonObjectRequest jsonObjectRequest = new JsonObjectRequest(
                    Request.Method.POST,
                    MainActivity.api_address + "update-destination",
                    jsonObject,
                    new Response.Listener<JSONObject>() {
                        @Override
                        public void onResponse(JSONObject response) {
                            Intent intent = new Intent(getApplicationContext(), DestLoadingActivity.class);
                            intent.putExtra("rid", rid);
                            startActivity(intent);
                            finish();
//                            Toast.makeText(getApplicationContext(), "Wait until arrival to destination.", Toast.LENGTH_SHORT).show();
                        }
                    }, new Response.ErrorListener() {
                @Override
                public void onErrorResponse(VolleyError error) {
                    Toast.makeText(getApplicationContext(), "Couldn't establish a connection with the server.", Toast.LENGTH_SHORT).show();
                }
            }
            );
            requestQueue.add(jsonObjectRequest);
        }
    }
}