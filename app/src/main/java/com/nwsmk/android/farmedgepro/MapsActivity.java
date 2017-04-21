package com.nwsmk.android.farmedgepro;

import android.Manifest;
import android.app.ProgressDialog;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.os.AsyncTask;
import android.support.v4.app.ActivityCompat;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.PolygonOptions;
import com.google.maps.android.SphericalUtil;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static com.nwsmk.android.farmedgepro.Utils.mean;
import static com.nwsmk.android.farmedgepro.Utils.sum2D;
import static java.lang.Math.abs;
import static java.lang.Math.floor;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.round;

public class MapsActivity extends FragmentActivity implements OnMapReadyCallback {

    private static final String TAG = MapsActivity.class.getSimpleName();
    private static final int PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION = 0x01;

    private GoogleMap mMap;
    private Point searchPoint;

    private Button mBtnDetect;
    private ProgressDialog mProgressDialog;

    private BaseLoaderCallback mBaseLoaderCallback = new BaseLoaderCallback(this) {

        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i("OpenCV", "OpenCV loaded sucessfully");
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_maps);
        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mBaseLoaderCallback);
        } else {
            Log.d("OpenCV", "OpenCV library found inside package. Using it!");
            mBaseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }


    /**
     * Manipulates the map once available.
     * This callback is triggered when the map is ready to be used.
     * This is where we can add markers or lines, add listeners or move the camera. In this case,
     * we just add a marker near Sydney, Australia.
     * If Google Play services is not installed on the device, the user will be prompted to install
     * it inside the SupportMapFragment. This method will only be triggered once the user has
     * installed Google Play services and returned to the app.
     */
    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;

        /** Map setup */
        // set enable self position
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                    PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION);
            return;
        }
        mMap.setMyLocationEnabled(true);

        // set map type to satellite imagery
        mMap.setMapType(GoogleMap.MAP_TYPE_SATELLITE);

        // Move marker to Bangkok
        LatLng bkk = new LatLng(13.7563, 100.5018);
        mMap.moveCamera(CameraUpdateFactory.newLatLng(bkk));

        // Set search point via long click on map
        mMap.setOnMapLongClickListener(new GoogleMap.OnMapLongClickListener() {
            @Override
            public void onMapLongClick(LatLng latLng) {
                mMap.clear();
                searchPoint = getPixelFromLatLng(latLng);
                mMap.addMarker(new MarkerOptions()
                        .position(latLng)
                        .title("Search Area"));
            }
        });

        // progress dialog
        mProgressDialog = new ProgressDialog(this);

        // show detect edges button
        mBtnDetect = (Button) findViewById(R.id.btn_detect);
        mBtnDetect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // start edge detection process
                mProgressDialog.setMessage("Starting process...");
                mProgressDialog.setCancelable(false);
                mProgressDialog.show();

                mMap.clear();

                // start capturing images
                GoogleMap.SnapshotReadyCallback callback = new GoogleMap.SnapshotReadyCallback() {
                    Bitmap bmp;

                    @Override
                    public void onSnapshotReady(Bitmap snapshot) {
                        bmp = snapshot;

                        try {
                            FileOutputStream out = new FileOutputStream("/mnt/sdcard/Pictures/out.png");
                            bmp.compress(Bitmap.CompressFormat.PNG, 90, out);

                            new EdgeDetector().execute();

                        } catch (IOException e) {

                        }
                    }
                };
                mMap.snapshot(callback);
            }
        });
    }

    // draw contour on map
    private void drawContourOnMap(MatOfPoint contour) {

        Point[] points = contour.toArray();
        LatLng tmpLatLng;

        PolygonOptions polygonOptions = new PolygonOptions();

        Log.d(TAG, "FINAL POLYGON LEN: " + points.length);

        if (points.length > 0) {
            for (int i = 0; i < points.length; i++) {
                tmpLatLng = getLatLngFromPixel(points[i]);
                mMap.addMarker(new MarkerOptions().position(tmpLatLng));
                polygonOptions.add(tmpLatLng);
            }
            polygonOptions.fillColor(Color.argb(40, 128, 156, 247));
            mMap.addPolygon(polygonOptions);

            // add area info
            tmpLatLng = getLatLngFromPixel(points[0]);
            double calAreaSqm = SphericalUtil.computeArea(polygonOptions.getPoints());
            // find rai
            int rai = (int) floor(calAreaSqm / 1600);
            int residual_rai = (int) (calAreaSqm - (rai*1600));
            int ngan = (int) floor(residual_rai / 400);
            int residual_ngan = (int) (residual_rai - (ngan*400));
            int wa = (int) floor(residual_ngan/4);

            String sTitle = "Total area: " + calAreaSqm + " sq.m.";
            String sCalAreaSqm = "Area: Rai = " + rai + ", Ngan = " + ngan + ", Tarang-wa = " + wa;
            mMap.addMarker(new MarkerOptions().position(tmpLatLng).title(sTitle).snippet(sCalAreaSqm)).showInfoWindow();

        } else {
            Toast.makeText(getApplicationContext(), "Cannot detect edges", Toast.LENGTH_SHORT).show();
        }
    }

    // draw contour on map
    private void drawContourOnMap(List<MatOfPoint2f> contours2f) {

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        for (int i = 0; i < contours2f.size(); i++) {
            contours.add(new MatOfPoint());
        }

        for (int i = 0; i < contours2f.size(); i++) {
            contours2f.get(i).convertTo(contours.get(i), CvType.CV_32S);
        }

        for (int i = 0; i < contours.size(); i++) {

            Point[] points = contours.get(i).toArray();
            LatLng tmpLatLng;

            PolygonOptions polygonOptions = new PolygonOptions();

            Log.d(TAG, "FINAL POLYGON LEN: " + points.length);

            if (points.length > 0) {
                for (int j = 0; j < points.length; j++) {
                    tmpLatLng = getLatLngFromPixel(points[j]);
                    mMap.addMarker(new MarkerOptions().position(tmpLatLng));
                    polygonOptions.add(tmpLatLng);
                }
                polygonOptions.fillColor(Color.argb(40, 128, 156, 247));
                mMap.addPolygon(polygonOptions);
            } else {
                Toast.makeText(getApplicationContext(), "Cannot detect edges", Toast.LENGTH_SHORT).show();
            }

        }

    }

    // get pixel coordinates from lat/lng input
    private Point getPixelFromLatLng(LatLng latLng) {
        android.graphics.Point pixelPoint = mMap.getProjection().toScreenLocation(latLng);
        Point openCVPoint = new Point(pixelPoint.x, pixelPoint.y);

        return openCVPoint;
    }

    // get lat/lng coordinates from point input
    private LatLng getLatLngFromPixel(Point point) {

        android.graphics.Point pixelPoint = new android.graphics.Point((int)point.x, (int)point.y);

        LatLng latLng = mMap.getProjection().fromScreenLocation(pixelPoint);

        return latLng;
    }

    private class EdgeDetector extends AsyncTask<Void, String, Void> {

        MatOfPoint encloseContour = new MatOfPoint();
        List<MatOfPoint> validContours;
        List<MatOfPoint2f> approxContours;

        private Bitmap debugBmp;

        public EdgeDetector() {
        }

        @Override
        protected void onPreExecute() {
            super.onPreExecute();
        }

        @Override
        protected Void doInBackground(Void... arg0) {

            publishProgress("Region Growing...");
            Mat growMat = grow(searchPoint);

            publishProgress("Converting to grayscale image...");
            /** PRO VERSION Mat grayMat = getGrayScaleMat(); */
            //Mat grayMat = getGrayScaleMat(growMat);

            //publishProgress("Performing floodfill image segmentation...");
            //Mat floodFillMat = getFloodFillMat(grayMat);

            publishProgress("Performing Canny edge detection...");
            //Mat cannyMat = getCannyMat(growMat, 50, 90);
            //Mat cannyMat = getCannyMat(growMat, 10, 100);
            //Mat cannyMat = getCannyMat(grayMat, 10, 100);
            //Mat cannyMat = getCannyMat(floodFillMat, 10, 100);

            publishProgress("Performing Hough line transform...");
            //Mat houghLineMat = getHoughLinesMat(cannyMat, 90, 100, 60);

            publishProgress("Performing contour detection...");
            validContours = getContours(growMat, 0.1f);
            //validContours = getContours(houghLineMat, 0.1f);
            //validContours = getContours(cannyMat, 0.1f);

            String numContours = Integer.toString(validContours.size());
            publishProgress("Processing contours: " + numContours + " ...");
            approxContours = getApproxPolyContours(validContours);

            publishProgress("Searching for enclose contour...");
            encloseContour = getEncloseContour(approxContours);

            return null;
        }

        @Override
        protected void onProgressUpdate(String... progress) {
            mProgressDialog.setMessage(progress[0]);
        }

        @Override
        protected void onPostExecute(Void result) {
            mProgressDialog.dismiss();
            drawContourOnMap(encloseContour);
            //drawContourOnMap(approxContours);
        }

        /** Region grow */
        private Mat grow(Point input_point) {

            Mat final_mat = new Mat();

            int mean_threshold = 5;

            // load image
            String filename = "/mnt/sdcard/Pictures/out.png";

            try {
                InputStream in = new FileInputStream(filename);

                // process image
                Bitmap originalBmp = BitmapFactory.decodeStream(in);
                debugBmp = originalBmp;

                int num_rows = originalBmp.getHeight();
                int num_cols = originalBmp.getWidth();

                Mat originalMat = new Mat(num_rows,
                        num_cols,
                        CvType.CV_8UC4,
                        new Scalar(0));
                Utils.bitmapToMat(originalBmp, originalMat);

                /** Box */
                // box size in pixels
                //int box_size = 8;
                int box_size = 4;
                int box_rows = num_rows/box_size;
                int box_cols = num_cols/box_size;
                // calculate features
                double[][] feat_mean = new double[box_rows][box_cols];
                for (int i = 0; i < box_rows; i++) {
                    for (int j = 0; j < box_cols; j++) {
                        int start_row = i*box_size;
                        int start_col = j*box_size;
                        double avg_mean = 0;
                        int    num_px   = 0;
                        for (int k = start_row; k < start_row + box_size; k++) {
                            for (int m = start_col; m < start_col + box_size; m++) {
                                double[] px_rgb = originalMat.get(k, m);
                                double tmp_mean = com.nwsmk.android.farmedgepro.Utils.mean(px_rgb);
                                avg_mean = ((num_px*avg_mean) + (1*tmp_mean))/ (1 + num_px);
                                num_px++;
                            }
                        }

                        feat_mean[i][j] = avg_mean;
                    }
                }

                // seed point info
                int seed_row = (int) input_point.y/box_size;
                int seed_col = (int) input_point.x/box_size;

                // edge
                int edge_index     = 1;
                int[][] edge_list  = new int[box_rows*box_cols][2];
                edge_list[0][0]    = seed_col;
                edge_list[0][1]    = seed_row;

                // neighbor
                int neigh_index    = 0;
                int[][] neigh_list = new int[box_rows*box_cols][2];
                int[][] neigh_mask = new int[4][2];
                neigh_mask[0][0] =  0; neigh_mask[0][1] = -1;
                neigh_mask[1][0] =  0; neigh_mask[1][1] =  1;
                neigh_mask[2][0] =  1; neigh_mask[2][1] =  0;
                neigh_mask[3][0] = -1; neigh_mask[3][1] =  0;

                // region
                double mean_region = feat_mean[seed_row][seed_col];
                int num_members    = 1;

                // flags
                int[][] box_flag = new int[box_rows][box_cols];
                //int[][] img_flag = new int[num_rows][num_cols];

                // output
                int[][] box_out  = new int[box_rows][box_cols];
                int[][] img_out  = new int[num_rows][num_cols];


                //int currx = seed_col;
                //int curry = seed_row;

                // while ((sum2D(box_out) < (box_rows*box_cols)) && (currx > 10) && (curry > 10) && (currx < (box_cols-10)) && (curry < (box_rows-10)) && (edge_index > 0)) {
                while ((sum2D(box_out) < (box_rows*box_cols)) && (edge_index > 0)) {

                    // process all edge members
                    int total_edge = edge_index;
                    for (int k = 0; k < total_edge; k++) {

                        edge_index = edge_index - 1;
                        int[][] edge = new int[1][2];
                        edge[0][0]   = edge_list[edge_index][0];
                        edge[0][1]   = edge_list[edge_index][1];

                        // find all neighbors of current edge
                        for (int j = 0; j < neigh_mask.length; j++) {

                            int x = min(max((edge[0][0] + neigh_mask[j][0]),0),(box_cols-1));
                            int y = min(max((edge[0][1] + neigh_mask[j][1]),0),(box_rows-1));

                            int[][] new_neighbor = new int[1][2];
                            new_neighbor[0][0]   = x;
                            new_neighbor[0][1]   = y;

                            //currx = x;
                            //curry = y;

                            // only add unprocessed neighbors to neighbor list
                            if (box_flag[y][x] == 0) {
                                box_flag[y][x] = 1;

                                // increase number of neighbor by one
                                neigh_list[neigh_index][0] = x;
                                neigh_list[neigh_index][1] = y;
                                neigh_index = neigh_index + 1;
                            }

                        }

                        // clear processed edge list
                        edge_list[edge_index][0] = 0;
                        edge_list[edge_index][1] = 0;
                    }

                    // process neighbors to get new edge
                    int total_neigh = neigh_index;
                    for (int j = 0; j < total_neigh; j++) {

                        // check if neighbor is in the region
                        // if YES, add this neighbor as a new edge
                        neigh_index = neigh_index - 1;

                        int x = neigh_list[neigh_index][0];
                        int y = neigh_list[neigh_index][1];

                        double mean_diff = abs(feat_mean[y][x]-mean_region);

                        //Log.d("Mean Diff = ", Double.toString(mean_diff));

                        if (mean_diff < mean_threshold) {

                            // update region mean
                            mean_region = ((1*feat_mean[y][x]) + (num_members*mean_region)) / (1 + num_members);
                            num_members = num_members + 1;

                            // update output
                            box_out[y][x] = 2;

                            // update edge index
                            edge_list[edge_index][0] = x;
                            edge_list[edge_index][1] = y;
                            edge_index = edge_index + 1;
                        }

                        // clear processed neighbor
                        neigh_list[neigh_index][0] = 0;
                        neigh_list[neigh_index][1] = 0;
                    }

                    //Log.d(TAG, Double.toString(sum2D(box_out)));
                }

                // finish region growing
                //Log.d(TAG, "END OF PROCESS");

                // see output image
                Mat out_mat = new Mat(num_rows, num_cols, CvType.CV_8U, new Scalar(0));
                for (int i = 0; i < box_rows; i++) {
                    for (int j = 0; j < box_cols; j++) {

                        if (box_out[i][j] == 2) {

                            int start_row = i * box_size;
                            int start_col = j * box_size;

                            for (int k = start_row; k < start_row + box_size; k++) {
                                for (int m = start_col; m < start_col + box_size; m++) {
                                    out_mat.put(k, m, 255);
                                }
                            }

                        }

                    }
                }

                // threshold
                Imgproc.threshold(out_mat, final_mat, 50, 255, Imgproc.THRESH_BINARY);

                Bitmap outBmp = originalBmp.copy(Bitmap.Config.ARGB_8888, false);
                Utils.matToBitmap(final_mat, outBmp);

                // for debugging purpose
                //final_mat.size();

            } catch (IOException ioe) {
                Log.e(TAG, "Error reading image file from local storage.");
            }

            return final_mat;
        }

        /** Image processing methods */
        // get grayscale image
        private Mat getGrayScaleMat(Mat inMat) {

            Mat grayMat = new Mat();

            /** PRO VERSION
            try {
                // load image
                InputStream in = new FileInputStream("/mnt/sdcard/Pictures/out.png");
                Bitmap originalBmp = BitmapFactory.decodeStream(in);

                // convert image to gray scale
                int bmpRow = originalBmp.getHeight();
                int bmpCol  = originalBmp.getWidth();

                Mat originalMat = new Mat(bmpRow, bmpCol, CvType.CV_8UC3, new Scalar(0));
                Utils.bitmapToMat(originalBmp, originalMat);

                Imgproc.cvtColor(originalMat, grayMat, Imgproc.COLOR_RGB2GRAY);

            } catch (IOException e) {
                Log.e(TAG, "IOException: " + e.getMessage());
            }
             **/
            Imgproc.cvtColor(inMat, grayMat, Imgproc.COLOR_RGB2GRAY);
            return grayMat;
        }

        // perform Canny edge detector
        private Mat getCannyMat(Mat inMat, int lowThreshold, int highThreshold) {

            Mat cannyMat = new Mat();

            Imgproc.Canny(inMat, cannyMat, lowThreshold, highThreshold);

            // for debugging purpose
            Bitmap outBmp = debugBmp.copy(Bitmap.Config.ARGB_8888, false);
            Utils.matToBitmap(cannyMat, outBmp);

            return cannyMat;
        }

        // perform Sobel edge detector
        private Mat getSobelMat(Mat inMat) {
            Mat sobelMat = new Mat();

            Mat grad_x = new Mat();
            Mat abs_grad_x = new Mat();
            Mat grad_y = new Mat();
            Mat abs_grad_y = new Mat();

            // Calculating gradient in horizontal direction
            Imgproc.Sobel(inMat, grad_x, CvType.CV_16S, 1, 0, 3, 1, 0);
            // Calculating gradient in vertical direction
            Imgproc.Sobel(inMat, grad_y, CvType.CV_16S, 0, 1, 3, 1, 0);
            // Calculating absolute gradient in both directions
            Core.convertScaleAbs(grad_x, abs_grad_x);
            Core.convertScaleAbs(grad_y, abs_grad_y);
            // Calculate resultant gradient
            Core.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 1, sobelMat);

            return sobelMat;
        }

        // perform Hough line transform
        private Mat getHoughLinesMat(Mat inMat, int minLineLen, int maxLineGap, int threshold) {
            Mat houghLinesMat;

            Mat lines = new Mat();

            Imgproc.HoughLinesP(inMat, lines, 1, Math.PI/180, threshold, minLineLen, maxLineGap);
            houghLinesMat = new Mat(inMat.rows(), inMat.cols(), CvType.CV_8UC1, new Scalar(0));

            for (int i=0; i<lines.rows(); i++) {
                // for (int i=0; i<lines.cols(); i++) {
                double[] points = lines.get(i, 0);
                // double[] points = lines.get(0,i);
                double x1, y1, x2, y2;

                x1 = points[0];
                y1 = points[1];
                x2 = points[2];
                y2 = points[3];

                Point pt1 = new Point(x1, y1);
                Point pt2 = new Point(x2, y2);

                // Draw lines on an image
                Imgproc.line(houghLinesMat, pt1, pt2, new Scalar(255, 0, 0), 2);
            }

            // for debugging purpose
            Bitmap outBmp = debugBmp.copy(Bitmap.Config.ARGB_8888, false);
            Utils.matToBitmap(houghLinesMat, outBmp);

            return houghLinesMat;
        }

        // perform contour detection
        private List<MatOfPoint> getContours(Mat inMat, float areaWeight) {

            List<MatOfPoint> contours = new ArrayList<>();

            // valid countours are contours with acceptable area
            //List<MatOfPoint> validContours;

            Mat hierarchy = new Mat();

            //Imgproc.findContours(inMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(inMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Mat drawing = new Mat(inMat.rows(), inMat.cols(), CvType.CV_8UC3, new Scalar(0));

            Random r = new Random();
            Log.d("CONTOUR COUNT", "No. of contours = " + contours.size());
            for (int i=0; i<contours.size(); i++) {
                Imgproc.drawContours(drawing, contours, i, new Scalar(r.nextInt(255), r.nextInt(255), r.nextInt(255)), -1);
            }

            // for debugging purpose
            Bitmap outBmp = debugBmp.copy(Bitmap.Config.ARGB_8888, false);
            Utils.matToBitmap(drawing, outBmp);

            // find max area
            double maxArea = getContourMaxArea(contours, hierarchy);

            // find valid contours
            validContours = getValidContours(contours, hierarchy, maxArea, areaWeight);

            return validContours;

        }

        // get maximum contour area
        private double getContourMaxArea(List<MatOfPoint> contours, Mat hierarchy) {

            double maxArea = 0;
            for (int i = 0; i < contours.size(); i++) {

                double[] contourInfo = hierarchy.get(0, i);
                int tmpChild = (int) contourInfo[2];

                if(tmpChild < 0) {
                    double tmpArea = Imgproc.contourArea(contours.get(i));
                    if (tmpArea > maxArea) {
                        maxArea = tmpArea;
                    }
                }
            }

            return maxArea;
        }

        // get valid contours
        private List<MatOfPoint> getValidContours(List<MatOfPoint> contours, Mat hierarchy, double maxArea, float weight) {

            List<MatOfPoint> validContours = new ArrayList<>();

            for (int i = 0; i < contours.size(); i++) {
                double[] contourInfo = hierarchy.get(0, i);
                int tmpChild = (int) contourInfo[2];
                if(tmpChild < 0) {
                    double tmp_area = Imgproc.contourArea(contours.get(i));
                    if (tmp_area >= weight*maxArea) {
                        validContours.add(contours.get(i));
                    }
                }
            }

            return validContours;
        }

        private List<MatOfPoint2f> getApproxPolyContours(List<MatOfPoint> contours) {

            // contours in 2D (float)
            List<MatOfPoint2f> contours2f     = new ArrayList<>();
            // approximated contours in 2D (float)
            List<MatOfPoint2f> polyMOP2f      = new ArrayList<>();


            // init list
            for (int i = 0; i < contours.size(); i++) {
                contours2f.add(new MatOfPoint2f());
                polyMOP2f.add(new MatOfPoint2f());
            }

            // convert contours into MatOfPoint2f
            for (int i = 0; i < contours.size(); i++) {
                contours.get(i).convertTo(contours2f.get(i), CvType.CV_32FC2);
                // contour in float
                Imgproc.approxPolyDP(contours2f.get(i), polyMOP2f.get(i), 10, true);
            }

            return polyMOP2f;
        }

        // search for a contours which encloses the search point
        private MatOfPoint getEncloseContour(List<MatOfPoint2f> polyMOP2f) {

            // approximated enclose contours (int)
            MatOfPoint polyMOP = new MatOfPoint();

            // convert contours into MatOfPoint2f
            for (int i = 0; i < polyMOP2f.size(); i++) {

                double ppt = Imgproc.pointPolygonTest(polyMOP2f.get(i), searchPoint, false);
                if (ppt >= 0) {
                    polyMOP2f.get(i).convertTo(polyMOP, CvType.CV_32S);
                }
            }

            return polyMOP;
        }


        /** TEST METHODS */
        private Mat getFloodFillMat(Mat inMat) {
            Mat floodFillMat = Mat.zeros(inMat.rows()+2, inMat.cols()+2, CvType.CV_8U);

            int flags = 4 + (255 << 8) + Imgproc.FLOODFILL_FIXED_RANGE;
            //int flags = Imgproc.FLOODFILL_MASK_ONLY;
            Imgproc.floodFill(inMat, floodFillMat, searchPoint, new Scalar(255, 255, 255),
                    new Rect(new Point(0,0), new Point(5,5)), new Scalar(15), new Scalar(15), flags);

            return floodFillMat;
        }
    }
}
