#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>
#include "laneDetection.h"
#include "calibration.h"
#include "functions.h"

using namespace cv;
using namespace std;
void videoSliderCallback(int, void*);

int sliderValue = 0;
//VideoCapture laneVideo;
Mat videoFrame; // Video Frame.
Mat videoFrameUndistorted; // Video Frame (after calibration).
Mat videoFramePerspective; // Video Frame (after perspective transform).
Mat _videoFrameUndistorted;
Mat debugWindow(540, 1280, CV_8UC3, Scalar(0,0,0)); //The Debug window.
Size videoSize; // Input Variable Size.
Mat cameraMatrix, dist; //Calibration Matrix.
Mat perspectiveMatrix; //Homography Matrix.
String coordinatetext = "";
Mat debugWindowROI;

//Point2f perspectiveSrc[] = {
//    Point2f(570,490),
//    Point2f(720,490),
//    Point2f(450,698),
//    Point2f(830,698)};

//Point2f perspectiveDst[] = {
//    Point2f(200,0),
//    Point2f(980,0),
//    Point2f(200,720),
//    Point2f(980,720)};



Point2f perspectiveSrc[] = {
    Point2f(570,300),
    Point2f(1000,300),
    Point2f(170,698),
    Point2f(1300,730)
};

Point2f perspectiveDst[] = {
    Point2f(0,0),
    Point2f(1280,0),
    Point2f(0,720),
    Point2f(1280,720)
};

void draw_lines(Mat frame){
    circle(frame,perspectiveSrc[0], 3, Scalar(0,255,0), FILLED);
    circle(frame,perspectiveSrc[1], 3, Scalar(0,255,0), FILLED);
    circle(frame,perspectiveSrc[3], 3, Scalar(0,255,0), FILLED);
    circle(frame,perspectiveSrc[2], 3, Scalar(0,255,0), FILLED);
    line(frame, perspectiveSrc[0], perspectiveSrc[1], Scalar(255,0,0), 1);
    line(frame, perspectiveSrc[1], perspectiveSrc[3], Scalar(255,0,0), 1);
    line(frame, perspectiveSrc[3], perspectiveSrc[2], Scalar(255,0,0), 1);
    line(frame, perspectiveSrc[2], perspectiveSrc[0], Scalar(255,0,0), 1);
}

funciones fun;


int main(int argc, char **argv)
{
    //Get the Perspective Matrix.
    perspectiveMatrix = getPerspectiveTransform(perspectiveSrc,perspectiveDst);


    string d = "jkj";
    string path_video = "/home/yrsn/Videos/video_.mp4";
    // *********************READ VIDEO*************************
    VideoCapture laneVideo("/home/yrsn/Videos/video_.mp4");



    videoSize = Size((int)laneVideo.get(CAP_PROP_FRAME_WIDTH),(int)laneVideo.get(CAP_PROP_FRAME_HEIGHT));

    std::string PATH_CAM = "/home/yrsn/Drone/VpFollow/cam/camera_cal/";

    //--------------Camera Calibration Start-----------------
    FileStorage fsRead;
    fsRead.open("Intrinsic.xml", FileStorage::READ);
    Mat src = imread(PATH_CAM+"calibration1.jpg");
    Mat dst;
    if (fsRead.isOpened() == false)
    {
        CameraCalibrator myCameraCalibrator(d,0);
        myCameraCalibrator.doCalibration(cameraMatrix, dist);
        FileStorage fs;
        fs.open("Intrinsic.xml", FileStorage::WRITE);
        fs << "CameraMatrix" << cameraMatrix;
        fs << "Dist" << dist;
        fs.release();
        fsRead.release();
        cout << "There is no existing intrinsic parameters XML file." << endl;
        cout << "Start calibraton......" << endl;
    }
    else
    {
        fsRead["CameraMatrix"] >> cameraMatrix;
        fsRead["Dist"] >> dist;
        fsRead.release();
    }
    undistort(src, dst, cameraMatrix, dist);
    //--------------Camera Calibration Finish-----------------

    //Display Video Image
    laneVideo.set(CAP_PROP_POS_FRAMES, 0);
    laneVideo >> videoFrame;
    //    imshow("video frame ",videoFrame);
    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    //    imshow("undis",videoFrameUndistorted);
//    _videoFrameUndistorted = videoFrameUndistorted.clone();
//    //Start Homography
//    warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, Size(370,465));

//    imshow("perspective",videoFramePerspective);


//    Mat mergeImage;
//    Mat finalResult;
//    //To create debug window
//    Mat debugWindowROI;
//    Mat resizePic;
//    //===========Start Real Time Processing===========
//    float laneDistant = 0;
//    stringstream ss;
//    namedWindow("Real Time Execution", WINDOW_NORMAL);
//    laneVideo.set(CAP_PROP_POS_FRAMES, 0);
//    laneVideo >> videoFrame;
//    Mat showVideos(videoFrame.size().height, videoFrame.size().width, CV_8UC3, Scalar(0,0,0));
//    laneDetection LaneAlgoVideo(_videoFrameUndistorted, perspectiveMatrix);
//    //imshow("enviarimage",_videoFrameUndistorted);

//    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
//    _videoFrameUndistorted = videoFrameUndistorted.clone();


    //    LaneAlgoVideo.laneDetctAlgo(" ");
    //    finalResult = LaneAlgoVideo.getFinalResult();


  //  fun.ReadVideo();

//    while(!videoFrame.empty())
//    {
//        draw_lines(videoFrame);
//        //Start Homography
//        warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix,videoSize); // videoSize

//        //Applying lane detection algorithm
//        //if(videoFrameCount == 0)
//        LaneAlgoVideo.laneDetctAlgo(" ");
//        finalResult = LaneAlgoVideo.getFinalResult();

//        //Detect the distance to lane center.
//        laneDistant = LaneAlgoVideo.getLaneCenterDist();
//        if(laneDistant > 0)
//        {
//            ss.str("");
//            ss.clear();
//            ss << abs(laneDistant) << "m " << " To the Right";
//            putText(finalResult, ss.str(), Point(50,50), 0, 2, Scalar(0, 0, 255), 2);
//        }
//        else
//        {
//            ss.str("");
//            ss.clear();
//            ss << abs(laneDistant) << "m " << " To the Left";
//            putText(finalResult, ss.str(), Point(50,50), 0, 2, Scalar(0, 0, 255), 2);
//        }

//        int twidth = videoFrame.size().width/2;
//        int theight = videoFrame.size().height/2;

//        debugWindowROI = showVideos(Rect(0,0,twidth,theight));
//        addWeighted(debugWindowROI, 0,fun.ResizeImage(videoFrame,0.5), 1, 0, debugWindowROI);

//        debugWindowROI = showVideos(Rect(twidth,0,twidth,theight));
//        addWeighted(debugWindowROI, 0,fun.ResizeImage(finalResult,0.5), 1, 0, debugWindowROI);

//        mergeImage = LaneAlgoVideo.getMergeImage().clone();
//        debugWindowROI = showVideos(Rect(0,theight,twidth,theight));
//        addWeighted(debugWindowROI, 0, fun.ResizeImage(mergeImage,0.5), 1, 0, debugWindowROI);

//        debugWindowROI = showVideos(Rect(twidth,theight,twidth,theight));
//        addWeighted(debugWindowROI, 0, fun.ResizeImage(videoFramePerspective,0.5), 1, 0, debugWindowROI);


//        imshow("Real Time Execution", showVideos);

//        laneVideo >> videoFrame;
//        if(videoFrame.empty()) break;

//        //videoFrameCount = (videoFrameCount + 1) % 10;

//        //Calibration
//        undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
//        _videoFrameUndistorted = videoFrameUndistorted.clone();
//        LaneAlgoVideo.setInputImage(_videoFrameUndistorted);

//        if(waitKey(10) == 27) break;
//    }
//    //    //===========Finish Real Time Processing===========

    waitKey(0);
    return 0;

}
