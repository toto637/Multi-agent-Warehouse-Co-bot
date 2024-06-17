package com.gp.maw;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;
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

public class MainActivity extends AppCompatActivity {

    public static final String api_address = "http://192.168.1.10:8000/";
    public final List<String> locations = new ArrayList<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        final RadioGroup radioGroup = findViewById(R.id.locationsRadioGroup);

        VolleySingleton volleySingleton = VolleySingleton.getInstance(getApplicationContext());
        RequestQueue requestQueue = volleySingleton.getRequestQueue();

        JsonObjectRequest jsonObjectRequest = new JsonObjectRequest(
                Request.Method.GET,
                api_address+"get-all-locations",
                null,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        try {
                            JSONArray locJSONArray = response.getJSONArray("locations");
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
        final EditText empId = findViewById(R.id.empID);
        final RadioGroup radioGroup = findViewById(R.id.locationsRadioGroup);

        int selectedId = radioGroup.getCheckedRadioButtonId();

        if (empId.getText().toString().equals("") || selectedId == -1) {
            Toast.makeText(getApplicationContext(), "Missing fields.", Toast.LENGTH_SHORT).show();
        } else {
            int id = Integer.parseInt(empId.getText().toString());

            if (id != 1111 && id != 2222 && id != 3333) {
                Toast.makeText(getApplicationContext(), "INVALID ID.", Toast.LENGTH_SHORT).show();
            } else {
                String location = locations.get(selectedId);
            JSONObject jsonObject = new JSONObject();
            try {
                jsonObject.put("id", id);
                jsonObject.put("location", location);
            } catch (JSONException e) {
                e.printStackTrace();
            }

            VolleySingleton volleySingleton = VolleySingleton.getInstance(getApplicationContext());
            RequestQueue requestQueue = volleySingleton.getRequestQueue();

            volleySingleton.setUid(id);

            JsonObjectRequest jsonObjectRequest = new JsonObjectRequest(
                    Request.Method.POST,
                    api_address+"submit-request",
                    jsonObject,
                    new Response.Listener<JSONObject>() {
                        @Override
                        public void onResponse(JSONObject response) {
                            Intent intent = new Intent(getApplicationContext(), LoadingActivity.class);
                            try {
                                if (response.getString("id").equals("-1")) {
                                    Toast.makeText(getApplicationContext(), "NOT Available at the moment, Please try again later.", Toast.LENGTH_SHORT).show();
                                } else {
                                    intent.putExtra("id", response.getString("id"));
                                    startActivity(intent);
                                    finish();
                                }
                            } catch (JSONException e) {
                                e.printStackTrace();
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
        }

    }
}