package com.gp.maw;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.Response;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.JsonObjectRequest;

import org.json.JSONException;
import org.json.JSONObject;

import HelperClasses.VolleySingleton;

public class DestLoadingActivity extends AppCompatActivity {

    String rid;

    TextView message;
    Button finishBtn;
    ProgressBar progressBar;

    private Handler handler = new Handler();
    private Runnable runnable = new Runnable() {
        @Override
        public void run() {
            // Update your TextView here
            updateTextView();
            // Schedule the next update after 30 seconds
            handler.postDelayed(this, 30000); // 30 seconds in milliseconds
        }
    };

    private void updateTextView() {
        VolleySingleton volleySingleton = VolleySingleton.getInstance(getApplicationContext());
        RequestQueue requestQueue = volleySingleton.getRequestQueue();

        JsonObjectRequest jsonObjectRequest = new JsonObjectRequest(
                Request.Method.GET,
                MainActivity.api_address+"time-to-destination/"+rid,
                null,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        try {
                            String time = response.getString("time");
                            float ftime = Float.parseFloat(time);
                            if (ftime > 0) {
                                message.setText("Time remaining to deliver order: " + time + " minutes.");
                            } else {
                                message.setText("Order Delivered!");
                                progressBar.setVisibility(View.INVISIBLE);
                                finishBtn.setVisibility(View.VISIBLE);
                                handler.removeCallbacks(runnable);
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

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_dest_loading);

        message = findViewById(R.id.message);
        finishBtn = findViewById(R.id.finishBtn);
        progressBar = findViewById(R.id.progressBar);

        Intent intent = getIntent();
        rid = intent.getStringExtra("rid");

        updateTextView();
    }

    public void restartJourney(View view) {
        clear_request_in_server();
        Intent intent = new Intent(getApplicationContext(), MainActivity.class);
        startActivity(intent);
        finish();
    }

    public void clear_request_in_server() {
        VolleySingleton volleySingleton = VolleySingleton.getInstance(getApplicationContext());
        RequestQueue requestQueue = volleySingleton.getRequestQueue();

        JsonObjectRequest jsonObjectRequest = new JsonObjectRequest(
                Request.Method.GET,
                MainActivity.api_address+"clear-request/"+rid,
                null,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {

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