/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller;

import android.app.ActionBar;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.hardware.usb.UsbManager;
import android.os.*;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.FrameLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.ftccommon.FtcRobotControllerService.FtcRobotControllerBinder;
import com.qualcomm.ftccommon.LaunchActivityConstantsList;
import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.ftcrobotcontroller.opmodes.FtcOpModeRegister;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.Dimmer;
import com.qualcomm.robotcore.util.ImmersiveMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.wifi.WifiDirectAssistant;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.Serializable;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.internal.*;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;


public class FtcRobotControllerActivity extends Activity implements View.OnTouchListener, CvCameraViewListener2{
  private static final String  TAG              = "IronReignFTC::Activity";
  private static final int REQUEST_CONFIG_WIFI_CHANNEL = 1;
  private static final boolean USE_DEVICE_EMULATION = false;
  private static final int NUM_GAMEPADS = 2;

  public static final String CONFIGURE_FILENAME = "CONFIGURE_FILENAME";

  static private int screenWidth;
  static private int screenHeight;
  static volatile public int blobx; //current x value of the centroid (center of mass) of the largest tracked blob contour
  static volatile public int bloby; //current y value of the centroid of the largest tracked blob
  static volatile public double maxContour = 0;
  static private double minContour = 1000; //smallest contour area that we will pay attention to
  static volatile public double targetContour = -1; //what is the size of the maximum contour just after it is selected by touch? - serves as the target size (distance to maintain from the object)
  private boolean               mIsColorSelected = false;
  private Mat                   mRgba;
  private Scalar                mBlobColorRgba;
  static volatile public Scalar mBlobColorHsv;
  private ColorBlobDetector     mDetector;
  private Mat                   mSpectrum;
  private Size                  SPECTRUM_SIZE;
  private Scalar                CONTOUR_COLOR;
  protected SharedPreferences preferences;
  private CameraBridgeViewBase mOpenCvCameraView;

  protected UpdateUI.Callback callback;
  protected Context context;
  private Utility utility;
  protected ImageButton buttonMenu;

  protected TextView textDeviceName;
  protected TextView textWifiDirectStatus;
  protected TextView textRobotStatus;
  protected TextView textWifiDirectPassphrase;
  protected TextView[] textGamepad = new TextView[NUM_GAMEPADS];
  protected TextView textOpMode;
  protected TextView textErrorMessage;
  protected ImmersiveMode immersion;

  protected SwerveUpdateUIHook updateUI;
  protected Dimmer dimmer;
  protected FrameLayout entireScreenLayout;

  protected FtcRobotControllerService controllerService;

  protected FtcEventLoop eventLoop;

  protected class RobotRestarter implements Restarter {

    public void requestRestart() {
      requestRobotRestart();
    }

  }

  private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
    @Override
    public void onManagerConnected(int status) {
      switch (status) {
        case LoaderCallbackInterface.SUCCESS:
        {
          Log.i(TAG, "OpenCV loaded successfully");
          mOpenCvCameraView.enableView();
          mOpenCvCameraView.setOnTouchListener(FtcRobotControllerActivity.this);
        } break;
        default:
        {
          super.onManagerConnected(status);
        } break;
      }
    }
  };

  protected ServiceConnection connection = new ServiceConnection() {
    @Override
    public void onServiceConnected(ComponentName name, IBinder service) {
      FtcRobotControllerBinder binder = (FtcRobotControllerBinder) service;
      onServiceBind(binder.getService());
    }

    @Override
    public void onServiceDisconnected(ComponentName name) {
      controllerService = null;
    }
  };


  @Override
  protected void onNewIntent(Intent intent) {
    super.onNewIntent(intent);
    if (UsbManager.ACTION_USB_ACCESSORY_ATTACHED.equals(intent.getAction())) {
      // a new USB device has been attached
      DbgLog.msg("USB Device attached; app restart may be needed");
    }
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    setContentView(R.layout.activity_ftc_controller);

    mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
    mOpenCvCameraView.setCvCameraViewListener(this);

    utility = new Utility(this);
    context = this;
    entireScreenLayout = (FrameLayout) findViewById(R.id.entire_screen);
    buttonMenu = (ImageButton) findViewById(R.id.menu_buttons);
    buttonMenu.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View v) {
        openOptionsMenu();
      }
    });

    textDeviceName = (TextView) findViewById(R.id.textDeviceName);
    textWifiDirectStatus = (TextView) findViewById(R.id.textWifiDirectStatus);
    textRobotStatus = (TextView) findViewById(R.id.textRobotStatus);
    textOpMode = (TextView) findViewById(R.id.textOpMode);
    textErrorMessage = (TextView) findViewById(R.id.textErrorMessage);
    textWifiDirectPassphrase = (TextView) findViewById(R.id.textWifiDirectPassphrase);
    textGamepad[0] = (TextView) findViewById(R.id.textGamepad1);
    textGamepad[1] = (TextView) findViewById(R.id.textGamepad2);
    immersion = new ImmersiveMode(getWindow().getDecorView());
    dimmer = new Dimmer(this);
    dimmer.longBright();
    Restarter restarter = new RobotRestarter();

    updateUI = new SwerveUpdateUIHook(this, dimmer);
    updateUI.setRestarter(restarter);
    updateUI.setTextViews(textWifiDirectStatus, textRobotStatus,
            textGamepad, textOpMode, textErrorMessage, textDeviceName);
    callback = updateUI.new CallbackHook();

    PreferenceManager.setDefaultValues(this, R.xml.preferences, false);
    preferences = PreferenceManager.getDefaultSharedPreferences(this);

    hittingMenuButtonBrightensScreen();

    if (USE_DEVICE_EMULATION) { HardwareFactory.enableDeviceEmulation(); }
  }

  @Override
  protected void onStart() {
    super.onStart();

    // save 4MB of logcat to the SD card
    RobotLog.writeLogcatToDisk(this, 4 * 1024);

    Intent intent = new Intent(this, FtcRobotControllerService.class);
    bindService(intent, connection, Context.BIND_AUTO_CREATE);

    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);

    callback.wifiDirectUpdate(WifiDirectAssistant.Event.DISCONNECTED);
/*
    entireScreenLayout.set Listener(new View.OnTouchListener() {
      @Override
      public boolean onTouch(View v, MotionEvent event) {
        dimmer.handleDimTimer();
        CameraTouch(v, event); //pass through to opencv handling
        return false;
      }
    });*/

  }

  @Override
  protected void onResume() {

    super.onResume();
    if (!OpenCVLoader.initDebug()) {
      Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
      OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_11, this, mLoaderCallback);
    } else {
      Log.d(TAG, "OpenCV library found inside package. Using it!");
      mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
    }
  }

  @Override
  public void onPause() {

    super.onPause();
    if (mOpenCvCameraView != null)
      mOpenCvCameraView.disableView();
  }

  @Override
  protected void onStop() {
    super.onStop();

    if (controllerService != null) unbindService(connection);

    RobotLog.cancelWriteLogcatToDisk(this);
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus){
    super.onWindowFocusChanged(hasFocus);
    // When the window loses focus (e.g., the action overflow is shown),
    // cancel any pending hide action. When the window gains focus,
    // hide the system UI.
    if (hasFocus) {
      if (ImmersiveMode.apiOver19()){
        // Immersive flag only works on API 19 and above.
        immersion.hideSystemUI();
      }
    } else {
      immersion.cancelSystemUIHide();
    }
  }


  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
    getMenuInflater().inflate(R.menu.ftc_robot_controller, menu);
    return true;
  }

  @Override
  public boolean onOptionsItemSelected(MenuItem item) {
    int id = item.getItemId();
    if (id==R.id.action_restart_robot) {
        dimmer.handleDimTimer();
        Toast.makeText(context, "Restarting Robot", Toast.LENGTH_SHORT).show();
        requestRobotRestart();
        return true;
    }
    if (id==R.id.action_settings) {
        // The string to launch this activity must match what's in AndroidManifest of FtcCommon for this activity.
        Intent settingsIntent = new Intent("com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity.intent.action.Launch");
        startActivityForResult(settingsIntent, LaunchActivityConstantsList.FTC_ROBOT_CONTROLLER_ACTIVITY_CONFIGURE_ROBOT);
        return true;
    }
    if (id==R.id.action_about) {
        // The string to launch this activity must match what's in AndroidManifest of FtcCommon for this activity.
        Intent intent = new Intent("com.qualcomm.ftccommon.configuration.AboutActivity.intent.action.Launch");
        startActivity(intent);
        return true;
    }
    if (id==R.id.action_exit_app) {
        finish();
        return true;
    }
    if (id==R.id.action_view_logs) {
        // The string to launch this activity must match what's in AndroidManifest of FtcCommon for this activity.
        Intent viewLogsIntent = new Intent("com.qualcomm.ftccommon.ViewLogsActivity.intent.action.Launch");
        viewLogsIntent.putExtra(LaunchActivityConstantsList.VIEW_LOGS_ACTIVITY_FILENAME, RobotLog.getLogFilename(this));
        startActivity(viewLogsIntent);
        return true;
    }
    return super.onOptionsItemSelected(item);
  }

@Override
  public void onConfigurationChanged(Configuration newConfig) {
    super.onConfigurationChanged(newConfig);
    // don't destroy assets on screen rotation
  }

  @Override
  protected void onActivityResult(int request, int result, Intent intent) {
    if (request == REQUEST_CONFIG_WIFI_CHANNEL) {
      if (result == RESULT_OK) {
        Toast toast = Toast.makeText(context, "Configuration Complete", Toast.LENGTH_LONG);
        toast.setGravity(Gravity.CENTER, 0, 0);
        showToast(toast);
      }
    }
    if (request == LaunchActivityConstantsList.FTC_ROBOT_CONTROLLER_ACTIVITY_CONFIGURE_ROBOT) {
      if (result == RESULT_OK) {
        Serializable extra = intent.getSerializableExtra(FtcRobotControllerActivity.CONFIGURE_FILENAME);
        if (extra != null) {
          utility.saveToPreferences(extra.toString(), R.string.pref_hardware_config_filename);
          utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
        }
      }
    }
  }

  public void onServiceBind(FtcRobotControllerService service) {
    DbgLog.msg("Bound to Ftc Controller Service");
    controllerService = service;
    updateUI.setControllerService(controllerService);

    callback.wifiDirectUpdate(controllerService.getWifiDirectStatus());
    callback.robotUpdate(controllerService.getRobotStatus());
    requestRobotSetup();
  }

  private void requestRobotSetup() {
    if (controllerService == null) return;

    FileInputStream fis = fileSetup();
    // if we can't find the file, don't try and build the robot.
    if (fis == null) { return; }

    HardwareFactory factory;

    // Modern Robotics Factory for use with Modern Robotics hardware
    HardwareFactory modernRoboticsFactory = new HardwareFactory(context);
    modernRoboticsFactory.setXmlInputStream(fis);
    factory = modernRoboticsFactory;

    eventLoop = new SwerveFtcEventLoop(factory, new FtcOpModeRegister(), callback, this);

    controllerService.setCallback(callback);
    controllerService.setupRobot(eventLoop);
  }

  private FileInputStream fileSetup() {

    final String filename = Utility.CONFIG_FILES_DIR
        + utility.getFilenameFromPrefs(R.string.pref_hardware_config_filename, Utility.NO_FILE) + Utility.FILE_EXT;

    FileInputStream fis;
    try {
      fis = new FileInputStream(filename);
    } catch (FileNotFoundException e) {
      String msg = "Cannot open robot configuration file - " + filename;
      utility.complainToast(msg, context);
      DbgLog.msg(msg);
      utility.saveToPreferences(Utility.NO_FILE, R.string.pref_hardware_config_filename);
      fis = null;
    }
    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
    return fis;
  }

  private void requestRobotShutdown() {
    if (controllerService == null) return;
    controllerService.shutdownRobot();
  }

  private void requestRobotRestart() {
    requestRobotShutdown();
    requestRobotSetup();
  }

  protected void hittingMenuButtonBrightensScreen() {
    ActionBar actionBar = getActionBar();
    if (actionBar != null) {
      actionBar.addOnMenuVisibilityListener(new ActionBar.OnMenuVisibilityListener() {
        @Override
        public void onMenuVisibilityChanged(boolean isVisible) {
          if (isVisible) {
            dimmer.handleDimTimer();
          }
        }
      });
    }
  }

  public void showToast(final Toast toast) {
    runOnUiThread(new Runnable() {
      @Override
      public void run() {
        toast.show();
      }
    });
  }

    //==============================================================================================

    /**
     * We are being notified that the FTC robot controller activity is being shut down.
     *
     * In response, we choose here to also terminate the underlying process. While not normally
     * something one does in a well-behaved and fully-debugged Android app, we do it here in order
     * to make the controller behavior more robust and reliable: there are known scenarios in which
     * the controller can get into an inoperable state that *requires* that this underlying process
     * be terminated (that is, there's something in that process state that's stuck). While this can
     * be done manually with 'swiping' closed the process, relying on the user to do that is just
     * silly. By terminating the process here, that will happen automatically when the user chooses
     * 'Exit' from the menu.
     *
     * @see <a href="http://developer.android.com/reference/android/app/Activity.html">Activity Life Cycle</a>
     */

    @Override protected void onDestroy()
        {
        // Do required superclass stuff
        super.onDestroy();
          if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        // Commit suicide
        Log.i(SynchronousOpMode.LOGGING_TAG, "FtcRobotControllerActivity committing process suicide");
        int pid = android.os.Process.myPid();
        android.os.Process.killProcess(pid);
        }

  public void onCameraViewStarted(int width, int height) {
    mRgba = new Mat(height, width, CvType.CV_8UC4);
    mDetector = new ColorBlobDetector();
    mSpectrum = new Mat();
    mBlobColorRgba = new Scalar(255);
    mBlobColorHsv = new Scalar(255);
    SPECTRUM_SIZE = new Size(200, 64);
    CONTOUR_COLOR = new Scalar(0,255,0,255);
    screenHeight=height;
    screenWidth=width;
  }

  public void onCameraViewStopped() {
    mRgba.release();
  }

  public boolean onTouch(View v, MotionEvent event) {
    int cols = mRgba.cols();
    int rows = mRgba.rows();

    int xOffset = (mOpenCvCameraView.getWidth() - cols) / 2;
    int yOffset = (mOpenCvCameraView.getHeight() - rows) / 2;

    int x = (int)event.getX() - xOffset;
    int y = (int)event.getY() - yOffset;

    dimmer.handleDimTimer();

    Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");

    if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

    Rect touchedRect = new Rect();

    touchedRect.x = (x>4) ? x-4 : 0;
    touchedRect.y = (y>4) ? y-4 : 0;

    touchedRect.width = (x+4 < cols) ? x + 4 - touchedRect.x : cols - touchedRect.x;
    touchedRect.height = (y+4 < rows) ? y + 4 - touchedRect.y : rows - touchedRect.y;

    Mat touchedRegionRgba = mRgba.submat(touchedRect);

    Mat touchedRegionHsv = new Mat();
    Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

    // Calculate average color of touched region
    mBlobColorHsv = Core.sumElems(touchedRegionHsv);
    int pointCount = touchedRect.width*touchedRect.height;
    for (int i = 0; i < mBlobColorHsv.val.length; i++)
      mBlobColorHsv.val[i] /= pointCount;

    mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);

    Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
            ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");

    mDetector.setHsvColor(mBlobColorHsv);

    Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

    mIsColorSelected = true;
    targetContour = -1; //reset target contour so we get a fresh target size on each touch event

    touchedRegionRgba.release();
    touchedRegionHsv.release();

    return false;

  }
  public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
    mRgba = inputFrame.rgba();

    if (mIsColorSelected) {
      mDetector.process(mRgba);
      List<MatOfPoint> contours = mDetector.getContours();
      Log.e(TAG, "Contours count: " + contours.size());
      Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR, 3);

      //get the centroid (center of mass) and area for each contour

      List<Moments> mu = new ArrayList<Moments>(contours.size());
      maxContour=0;
      for (int i = 0; i < contours.size(); i++) {
        mu.add(i, Imgproc.moments(contours.get(i), false));
        Moments p = mu.get(i);
        int x = (int) (p.get_m10() / p.get_m00());
        int y = (int) (p.get_m01() / p.get_m00());
        //Core.circle(mRgba, new Point(x, y), 4, new Scalar(255,49,0,255));
        Core.circle(mRgba, new Point(x, y), 5, CONTOUR_COLOR, -1);
        double area = Imgproc.contourArea(contours.get(i));
        if (area > maxContour)
        {
          maxContour=area;
          blobx=x;
          bloby=y;
        }
      }

      if (targetContour == -1 && maxContour > 0 )
      {
        targetContour = maxContour; //new target size, thus distance to object
      }




      Mat colorLabel = mRgba.submat(4, 68, 4, 68);
      colorLabel.setTo(mBlobColorRgba);

      Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
      mSpectrum.copyTo(spectrumLabel);
    }

    return mRgba;
  }

  private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
    Mat pointMatRgba = new Mat();
    Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
    Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

    return new Scalar(pointMatRgba.get(0, 0));
  }


    //==============================================================================================
    // Hooking infrastructure (Swerve)
    //
    // The code below has been added to the stock FtcRobotControllerActivity in order to hook
    // into state transitions of various kinds that happen within the robot controller application.
    // Most of what's here is of necessity pretty obscure and technical in nature, but
    // fortunately those details won't be of significance to most.

    static class SwerveEventLoopMonitor implements EventLoopManager.EventLoopMonitor
    // Hook to receive event monitor state transition
        {
        //------------------------------------------------------------------------------------------
        // State
        //------------------------------------------------------------------------------------------

        // The previously installed hook
        final EventLoopManager.EventLoopMonitor prevMonitor;

        //------------------------------------------------------------------------------------------
        // Construction
        //------------------------------------------------------------------------------------------

        SwerveEventLoopMonitor(EventLoopManager.EventLoopMonitor prevMonitor)
            {
            this.prevMonitor = prevMonitor;
            }

        // Make sure we're installed in the in the hook of the current event loop
        public synchronized static boolean installIfNecessary(FtcRobotControllerService service)
            {
            if (service == null)
                return false;

            Robot robot = MemberUtil.robotOfFtcRobotControllerService(service);
            if (robot == null)
                return false;

            EventLoopManager eventLoopManager = MemberUtil.eventLoopManagerOfRobot(robot);
            if (eventLoopManager == null)
                return false;

            EventLoopManager.EventLoopMonitor monitor = MemberUtil.monitorOfEventLoopManager(eventLoopManager);
            if (monitor == null)
                return false;

            if (monitor instanceof SwerveEventLoopMonitor)
                {
                // we're already installed
                }
            else
                {
                SwerveEventLoopMonitor newMonitor = new SwerveEventLoopMonitor(monitor);
                eventLoopManager.setMonitor(newMonitor);
                }

            return true;
            }

        //------------------------------------------------------------------------------------------
        // Notifications
        //------------------------------------------------------------------------------------------

        @Override
        public void onStateChange(RobotState newState)
            {
            this.prevMonitor.onStateChange(newState);
            RobotStateTransitionNotifier.onRobotStateChange(newState);
            }

        }

    class SwerveUpdateUIHook extends UpdateUI
    // Hook used to augment the user interface
        {
        //------------------------------------------------------------------------------------------
        // State
        //------------------------------------------------------------------------------------------

        FtcRobotControllerActivity activity;
        FtcRobotControllerService  controllerService;

        //------------------------------------------------------------------------------------------
         // Construction
        //------------------------------------------------------------------------------------------

        SwerveUpdateUIHook(FtcRobotControllerActivity activity, Dimmer dimmer)
            {
            super(activity, dimmer);
            this.activity = activity;
            this.controllerService = null;
            }

        @Override
        public void setControllerService(FtcRobotControllerService controllerService)
            {
            super.setControllerService(controllerService);
            this.controllerService = controllerService;
            }

        //------------------------------------------------------------------------------------------
        // Operations
        //------------------------------------------------------------------------------------------

        class CallbackHook extends UpdateUI.Callback
            {
            //--------------------------------------------------------------------------------------
            // Operations
            //--------------------------------------------------------------------------------------

            @Override
            public void robotUpdate(final String status)
                {
                super.robotUpdate(status);
                RobotStateTransitionNotifier.onRobotUpdate(status);

                // Make sure we get to see all the robot state transitions
                SwerveEventLoopMonitor.installIfNecessary(controllerService);
                }

            @Override
            public void wifiDirectUpdate(WifiDirectAssistant.Event event)
                {
                super.wifiDirectUpdate(event);

                final String message = controllerService == null
                        ? ""
                        : String.format("Wifi Direct passphrase: %s", controllerService.getWifiDirectAssistant().getPassphrase());

                SwerveUpdateUIHook.this.activity.runOnUiThread(new Runnable()
                    {
                    @Override public void run()
                        {
                        activity.textWifiDirectPassphrase.setText(message);
                        }
                    });
                }
            }
        }
  }




















