package com.graphhopper.android.mfsensor;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.os.Bundle;
import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.graphhopper.GraphHopper;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.AlgorithmOptions;
import com.graphhopper.routing.util.CarFlagEncoder;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.FastestWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.GPXEntry;
import com.graphhopper.util.Parameters;

import java.util.List;

import deadreckoning.R;
import matching.EdgeMatch;
import matching.GPXFile;
import matching.MapMatching;
import matching.MatchResult;


public class Matcher extends AppCompatActivity {

    double[] res = new double[2];


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Button return_matcher = findViewById(R.id.return_matcher);

        res = matcher();

        return_matcher.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = new Intent();
                intent.putExtra("data_return", res);
                setResult(RESULT_OK,intent);
                finish();
            }
        });


    }
    public void onBackPressed(){
        res = matcher();
        Intent intents = new Intent();
        intents.putExtra("data_return",res);
        setResult(RESULT_OK,intents);
        finish();
    }

    public double[] matcher(){
        GraphHopper hopper = new GraphHopperOSM();
        //hopper.setDataReaderFile("/mnt/sdcard/here.osm");
        hopper.setGraphHopperLocation("/mnt/sdcard/Download/graph-cache");
        CarFlagEncoder encoder = new CarFlagEncoder();
        hopper.setEncodingManager(new EncodingManager(encoder));
        hopper.getCHFactoryDecorator().setEnabled(false);
        hopper.setMinNetworkSize(4,2);
        hopper.importOrLoad();

        // create MapMatching object, can and should be shared accross threads
        String algorithm = Parameters.Algorithms.DIJKSTRA_BI;
        Weighting weighting = new FastestWeighting(encoder);
        AlgorithmOptions algoOptions = new AlgorithmOptions(algorithm, weighting);
        MapMatching mapMatching = new MapMatching(hopper, algoOptions);

// do the actual matching, get the GPX entries from a file or via stream

//        List<GPXEntry> inputGPXEntries = new GPXFile().doImport(Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + getString(R.string.app_name) + ".gpx").getEntries();
        @SuppressLint("SdCardPath") List<GPXEntry> inputGPXEntries = new GPXFile().doImport("/mnt/sdcard/Download/3.gpx").getEntries();
        MatchResult mr = mapMatching.doWork(inputGPXEntries);

// return GraphHopper edges with all associated GPX entries
        List<EdgeMatch> matches = mr.getEdgeMatches();
// now do something with the edges like storing the edgeIds or doing fetchWayGeometry etc
        Toast.makeText(Matcher.this,"Matched!",Toast.LENGTH_SHORT).show();
        double[] res  = new double[2];
        res[0] = matches.get(matches.size()-1).getGpxExtensions().get(0).getEntry().getLon();
        res[1] = matches.get(matches.size()-1).getGpxExtensions().get(0).getEntry().getLat();
        return res;

    }
}
