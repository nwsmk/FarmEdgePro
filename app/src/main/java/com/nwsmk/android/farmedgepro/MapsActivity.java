package com.nwsmk.android.farmedgepro;

import android.app.ProgressDialog;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.os.AsyncTask;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.PolygonOptions;

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
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class MapsActivity extends FragmentActivity implements OnMapReadyCallback {

    private static final String TAG = MapsActivity.class.getSimpleName();

    private GoogleMap mMap;
    private Point searchPoint;

    private Button mBtnDetect;

    private BaseLoaderCallback mBaseLoaderCallback = new BaseLoaderCallback(this) {

        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i("OpenCV", "OpenCV loaded sucessfully");
                } break;
                default: {
                    super.onManagerConnected(status);
                } break;
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

        /** Inflates UI */
        // show detect edges button
        mBtnDetect = (Button) findViewById(R.id.btn_detect);
        mBtnDetect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // start capturing images
                GoogleMap.SnapshotReadyCallback callback = new GoogleMap.SnapshotReadyCallback() {
                    Bitmap bmp;

                    @Override
                    public void onSnapshotReady(Bitmap snapshot) {
                        bmp = snapshot;

                        try {
                            FileOutputStream out = new FileOutputStream("/mnt/sdcard/Pictures/out.png");
                            bmp.compress(Bitmap.CompressFormat.PNG, 90, out);

                            // start edge detection process
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
        } else {
            Toast.makeText(getApplicationContext(), "Cannot detect edges", Toast.LENGTH_SHORT).show();
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

    private class EdgeDetector extends AsyncTask<Void, Void, Void> {

        MatOfPoint encloseContour = new MatOfPoint();

        private ProgressDialog mProgressDialog = new ProgressDialog(MapsActivity.this);

        public EdgeDetector() {
        }

        @Override
        protected void onPreExecute() {
            super.onPreExecute();
            mProgressDialog.setMessage("Starting process...");
        }

        @Override
        protected Void doInBackground(Void... arg0) {

            mProgressDialog.setMessage("Converting to grayscale image...");
            Mat grayMat = getGrayScaleMat();

            mProgressDialog.setMessage("Performing Canny edge detection...");
            Mat cannyMat = getCannyMat(grayMat, 10, 100);

            mProgressDialog.setMessage("Performing Hough line transform...");
            Mat houghLineMat = getHoughLinesMat(cannyMat, 90, 100, 60);

            mProgressDialog.setMessage("Performing contour detection...");
            List<MatOfPoint> validContours = getContours(houghLineMat, 0.2f);

            String numContours = Integer.toString(validContours.size());
            mProgressDialog.setMessage("Processing contours: " + numContours + " ...");
            List<MatOfPoint2f> approxContours = getApproxPolyContours(validContours);

            mProgressDialog.setMessage("Searching for enclose contour...");
            encloseContour = getEncloseContour(approxContours);

            return null;
        }

        @Override
        protected void onPostExecute(Void result) {
            mProgressDialog.dismiss();
            drawContourOnMap(encloseContour);
        }

        /** Image processing methods */
        // get grayscale image
        private Mat getGrayScaleMat() {

            Mat grayMat = new Mat();

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

            return grayMat;
        }

        // perform Canny edge detector
        private Mat getCannyMat(Mat inMat, int lowThreshold, int highThreshold) {

            Mat cannyMat = new Mat();

            Imgproc.Canny(inMat, cannyMat, lowThreshold, highThreshold);

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

            return houghLinesMat;
        }

        // perform contour detection
        private List<MatOfPoint> getContours(Mat inMat, float areaWeight) {

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

            // valid countours are contours with acceptable area
            List<MatOfPoint> validContours = new ArrayList<MatOfPoint>();

            Mat hierarchy = new Mat();

            Imgproc.findContours(inMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Mat drawing = new Mat(inMat.rows(), inMat.cols(), CvType.CV_8UC3, new Scalar(0));

            Random r = new Random();
            Log.d("CONTOUR COUNT", "No. of contours = " + contours.size());
            for (int i=0; i<contours.size(); i++) {
                Imgproc.drawContours(drawing, contours, i, new Scalar(r.nextInt(255), r.nextInt(255), r.nextInt(255)), -1);
            }

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

            List<MatOfPoint> validContours = new ArrayList<MatOfPoint>();

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
            List<MatOfPoint2f> contours2f     = new ArrayList<MatOfPoint2f>();
            // approximated contours in 2D (float)
            List<MatOfPoint2f> polyMOP2f      = new ArrayList<MatOfPoint2f>();


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
    }
}
