package com.gp.maw;

import androidx.activity.result.ActivityResultLauncher;
import androidx.appcompat.app.AppCompatActivity;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.Response;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.JsonObjectRequest;
import com.android.volley.toolbox.Volley;

import org.json.JSONException;
import org.json.JSONObject;

import com.google.zxing.integration.android.IntentIntegrator;
import com.google.zxing.integration.android.IntentResult;
import com.journeyapps.barcodescanner.ScanContract;
import com.journeyapps.barcodescanner.ScanOptions;

import HelperClasses.VolleySingleton;
import QRNeededClasses.CaptureAct;

public class LoadingActivity extends AppCompatActivity {

    int rid;

    TextView toa;
    Button orderBtn;

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
                MainActivity.api_address+"get-time-of-arrival/"+rid,
                null,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        try {
                            String time = response.getString("time");
                            float ftime = Float.parseFloat(time);
                            if (ftime > 0) {
                                toa.setText("The estimated time of arrival: " + time + " minutes.");
                            } else {
                                toa.setText("Arrived!");
                                orderBtn.setVisibility(View.VISIBLE);
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
        setContentView(R.layout.activity_loading);

        toa = findViewById(R.id.ToA);
        orderBtn = findViewById(R.id.orderBtn);

        Intent intent = getIntent();
        if (intent != null) {
            rid = Integer.parseInt(intent.getStringExtra("id"));
//            String time = intent.getStringExtra("time");
//            toa.setText("The estimated time of arrival: "+time+" minutes.");
            updateTextView();
        }

        handler.postDelayed(runnable, 30000);
    }

    public void scanCode(View view) {
        ScanOptions options = new ScanOptions();
        options.setPrompt("Volume up to flash on");
        options.setBeepEnabled(true);
        options.setOrientationLocked(true);
        options.setCaptureActivity(CaptureAct.class);
        cl.launch(options);
    }

    ActivityResultLauncher<ScanOptions> cl = registerForActivityResult(new ScanContract(), result -> {
        if(result.getContents()!=null)
        {
            Intent intent = new Intent(getApplicationContext(), DestActivity.class);
            intent.putExtra("id", result.getContents().trim());
            startActivity(intent);
            finish();
        }
    });
}