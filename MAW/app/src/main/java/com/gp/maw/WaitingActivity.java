package com.gp.maw;


import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.Response;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.JsonObjectRequest;

import org.json.JSONException;
import org.json.JSONObject;

import HelperClasses.VolleySingleton;

public class WaitingActivity  extends AppCompatActivity {
    private Handler handler = new Handler();
    private Runnable runnable = new Runnable() {
        @Override
        public void run() {
            getRId();
            handler.postDelayed(this, 10000);
        }
    };

    public void getRId() {
        JSONObject jsonObject = new JSONObject();
        Intent intent = getIntent();
        try {
            jsonObject.put("id", Integer.parseInt(intent.getStringExtra("id")));
            jsonObject.put("location", intent.getStringExtra("location"));
        } catch (JSONException e) {
            e.printStackTrace();
        }

        VolleySingleton volleySingleton = VolleySingleton.getInstance(getApplicationContext());
        RequestQueue requestQueue = volleySingleton.getRequestQueue();

        JsonObjectRequest jsonObjectRequest = new JsonObjectRequest(
                Request.Method.POST,
                MainActivity.api_address + "submit-request",
                jsonObject,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        Intent intent = new Intent(getApplicationContext(), LoadingActivity.class);
                        try {
                            if (!response.getString("id").equals("-1")) {
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

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_waiting);

        getRId();
        handler.postDelayed(runnable, 10000);
    }
}
