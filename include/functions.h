//class functions
#include<opencv2/calib3d.hpp>
#include<opencv2/core/types.hpp>
#include<opencv2/core/persistence.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/core/types_c.h>
#include<iostream>
using namespace cv;
using namespace std;

class funciones{

 public:

    void ReadVideo()
    {
        Point2f perspectiveSrc[] = {
            Point2f(570,300),
            Point2f(1000,300),
            Point2f(170,698),
            Point2f(1300,698)
        };

        Point2f perspectiveDst[] = {
            Point2f(0,0),
            Point2f(1280,0),
            Point2f(0,720),
            Point2f(1280,720)
        };
       Mat temp,temp1;
        VideoCapture cap("/home/yrsn/Videos/video_.mp4");
        int lowH = 0; int highH = 26;
        int lowS = 98; int highS = 118;
        int lowV = 71;  int highV = 183;
        while(1){
           Mat frame;
           Mat frame1;
           Mat output;
           Mat edgeImage;
           cap >> frame;
           if (frame.empty())
             break;

           frame1  = frame.clone();

           circle(frame1,perspectiveSrc[0], 3, Scalar(0,255,0), FILLED);
           circle(frame1,perspectiveSrc[1], 3, Scalar(0,255,0), FILLED);
           circle(frame1,perspectiveSrc[3], 3, Scalar(0,255,0), FILLED);
           circle(frame1,perspectiveSrc[2], 3, Scalar(0,255,0), FILLED);
           line(frame1, perspectiveSrc[0], perspectiveSrc[1], Scalar(255,0,0), 1);
           line(frame1, perspectiveSrc[1], perspectiveSrc[3], Scalar(255,0,0), 1);
           line(frame1, perspectiveSrc[3], perspectiveSrc[2], Scalar(255,0,0), 1);
           line(frame1, perspectiveSrc[2], perspectiveSrc[0], Scalar(255,0,0), 1);


           Mat Matrix = getPerspectiveTransform(perspectiveSrc,perspectiveDst);
           warpPerspective(frame, output, Matrix,Size(1280,720));


           //temp = bordes(frame,62);
           //temp1 = dilatacion(temp,1.5);
           imshow( "scena-with-roci estract", frame1);
          // imshow( "original - video", frame);
           imshow( "perspective transforme", output);

           Mat imgHSV;
           cvtColor(output, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV color space

           Mat imgThresholded;
           inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imgThresholded);
           imshow("imagethres",imgThresholded);

           //bool check = imwrite("/home/yrsn/Dev/PFCII/img/line.jpg", output);

//           Canny(output, edgeImage, 100, 150, 3);
//           temp = bordes(output,40);
//           temp1 = dilatacion(temp,1);

           //imshow( "canny-filter", edgeImage);
           //imshow( "final result", temp1);
           //imshow( "Frame", frame);
           char c=(char)waitKey(25);
           if(c==27)
             break;
         }
         cap.release();
         destroyAllWindows();
    }

    Mat ResizeImage(Mat &image, float scale)
    {
        Mat resized;
        resize(image, resized, Size(image.cols*scale, image.rows*scale), INTER_LINEAR);
        return resized;
    }

    Mat bordes(Mat image,int level)
    {
       Mat gray,canny,dst,out;
       cvtColor(image,gray,COLOR_BGR2GRAY);
       blur(gray,canny,cv::Size(3,3));

       Canny(canny,out,level,level*3,3);
       dst.create(image.size(),image.type());
       return out;
    }

    Mat dilatacion(Mat image ,int dilation_size)
    {
       Mat out,dilation_dst;
       int dilation_type = MORPH_RECT;

       Mat element = getStructuringElement(
             dilation_type,Size(2*dilation_size+1,2*dilation_size+1),
             Point(dilation_size,dilation_size)
             );
       dilate(image,out,element);
      return out;
    }
};

