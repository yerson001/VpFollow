// vanishpoint clas// vanishpoint classs
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ios>
#include <opencv2/imgproc/imgproc_c.h>
//#include "main_func.h"
#define CENTER_RANGE 0.1
#define PI 3.1415926

using namespace std;
using namespace cv;

class vanishpoint
{
public:

  bool hough_line_detect(Mat & image, Mat & cdst, vector<Vec2f> & left_lines, vector<Vec2f> & right_lines)
  {
    Mat dst;
    Canny(image, dst, 30, 70, 3);
    cvtColor(dst, cdst, COLOR_GRAY2BGR);

    vector<Vec2f> lines;
    // detect lines
    HoughLines(dst, lines, 1, CV_PI / 180, 150, 0, 0);
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        if (10 * PI / 180 < theta && theta < 60 * PI / 180) {
            left_lines.push_back(lines[i]);
          } else if (110 * PI / 180 < theta && theta < 170 * PI / 180) {
            right_lines.push_back(lines[i]);
          }
      }
    return true;
  }

  bool draw_line(Mat & image, vector<Vec2f> & vec_lines, Scalar color)
  {
    for (size_t i = 0; i < vec_lines.size(); ++i) {
        float rho = vec_lines[i][0], theta = vec_lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        cv::line(image, pt1, pt2, color, 3, LINE_AA);
      }
    return true;
  }


  Point get_vanish_point(vector<Point> & points, Mat & cdst)
  {
    long x = 0, y = 0;
    if (points.size() == 0) {
        return Point(0, 0);
      }
    for (size_t i = 0; i < points.size(); ++i) {
        x += points[i].x;
        y += points[i].y;
      }
    x /= points.size();
    y /= points.size();
    Point vp(x, y);
    circle(cdst, vp, 5, Scalar(0, 0, 0),5 );
    return vp;
  }

  Point vanish_point_detection(Mat & image, Mat & cdst)
  {
    Mat dst;
    vector<Vec2f> left_lines;
    vector<Point> Intersection;
    vector<Vec2f> right_lines;
    hough_line_detect(image, cdst, left_lines, right_lines);
    draw_line(cdst, left_lines, Scalar(0, 255, 0));
    draw_line(cdst, right_lines, Scalar(255, 0, 0));
    size_t i = 0, j = 0;
    long sum_x = 0;
    long sum_y = 0;
    double x = 0;
    double y = 0;
    Canny(image, dst, 30, 70, 3);
    cvtColor(dst, cdst, COLOR_GRAY2BGR);

    vector<Vec2f> lines;
    //detect lines
    HoughLines(dst, lines, 1, CV_PI / 180, 150, 0, 0);

    //// draw lines
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        if (10 * PI / 180 < theta && theta < 60 * PI / 180) {
            line(image, pt1, pt2, Scalar(0, 255, 0), 3, LINE_AA);
          } else if (110 * PI / 180 < theta && theta < 170 * PI / 180) {
            line(image, pt1, pt2, Scalar(255, 0, 0), 3, LINE_AA);
          } else {
            line(image, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
          }
      }
    for (i = 0; i < left_lines.size(); ++i) {
        for (j = 0; j < right_lines.size(); ++j) {
            float rho_l = left_lines[i][0], theta_l = left_lines[i][1];
            float rho_r = right_lines[j][0], theta_r = right_lines[j][1];
            double denom = (sin(theta_l) * cos(theta_r) - cos(theta_l) * sin(theta_r));
            x = (rho_r * sin(theta_l) - rho_l * sin(theta_r)) / denom;
            y = (rho_l * cos(theta_r) - rho_r * cos(theta_l)) / denom;
            //cout<<"("<<x<<","<<y<<")"<<endl;
            Point pt(x, y);
            Intersection.push_back(pt);
            circle(image, pt, 5, Scalar(0, 0, 0),5 );
          }
      }

    return get_vanish_point(Intersection, image);
  }

  void init()
  {
    Point mark_point, current_point;
    int center_x_min, center_x_max, center_y_min, center_y_max;
    Mat image;
    Mat outImage;
    Mat dst, cdst;
    float size = 40;
    //cout<<"what percentage to reduce the image: "<<endl;
    //cin >>size;
    size = size/100;

//    double start = double(getTickCount());

    //VideoCapture cap("./Data/corredor7.wmv");
    VideoCapture cap("/home/yrsn/Videos/video_.mp4");

    cap >> image;
    resize(image, outImage, cv::Size(), size, size);

    mark_point = vanish_point_detection(outImage, cdst);
    center_x_max = (1 + CENTER_RANGE) * mark_point.x;
    center_x_min = (1 - CENTER_RANGE) * mark_point.x;
    center_y_max = (1 + CENTER_RANGE) * mark_point.y;
    center_y_min = (1 - CENTER_RANGE) * mark_point.y;

    while (char(waitKey(1)) != 27 && cap.isOpened()) {
        //start = double(getTickCount());
        cap >> outImage;
        resize(outImage, outImage, cv::Size(), size, size);
        if (outImage.empty()) {
            cout << "Video over" << endl;
            break;
          } else {
            current_point = vanish_point_detection(outImage, cdst);
             circle(outImage,current_point, 3, Scalar(60,233,239), FILLED);

//            double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
//            cout << "It took " << duration_ms << " ms." << endl;
            if (current_point.x > center_x_max) {
                cout << "left" << endl;
              } else if (current_point.x < center_x_min) {
                cout << "right" << endl;
              } else {
                cout << "center" << endl;
              }
            line(cdst, Point(mark_point.x, 0), Point(mark_point.x, outImage.rows), Scalar(10, 10, 10), 1, CV_AA);
            //video.write(cdst);
            imshow("detected lines", outImage);

          }

      }
  }

};
