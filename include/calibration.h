//camera calibration
#include<iostream>
#include<opencv2/calib3d.hpp>
#include<opencv2/core/types.hpp>
#include<opencv2/core/persistence.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/core/types_c.h>

class CameraCalibrator {
   public:
    CameraCalibrator(std::string dir,int dev):PATH(dir),device(dev)
    {
        NumImages = 20;
    }

    void setFilename();
    void addPoints();
    void doCalibration(cv::Mat &cameraMatrix, cv::Mat &dist);

   private:
    std::vector<std::string> m_filenames;
    std::vector<std::vector<cv::Point2f>> m_srcPoints;
    std::vector<std::vector<cv::Point3f>> m_dstpoints;
    cv::Size m_imageSize;
    std::string PATH;
    int device;
    int NumImages;
};


void CameraCalibrator::setFilename()
{
    m_filenames.clear();

    for(int i=0; i<NumImages; i++)
        m_filenames.push_back(PATH+"Cam"+std::to_string(i+1)+".jpg");
        //std::cout<<PATH+"calibration"+std::to_string(i+1)+".jpg"<<std::endl;
}

void CameraCalibrator::addPoints()
{
    std::vector<cv::Point2f> chessboardCorner;
    std::vector<cv::Point3f> realWorldCoord;
    cv::Mat image;

    for(int i=0; i<6; i++)
        for(int j=0; j<9; j++)
            realWorldCoord.push_back(cv::Point3f(i,j,0.0f));

    // find chessboard 2d coordinates
    for(int i=0; i<m_filenames.size(); i++)
        image = cv::imread(m_filenames[i],cv::IMREAD_GRAYSCALE);
        if(chessboardCorner.size() == 54)
        {
            m_dstpoints.push_back(realWorldCoord);
            m_srcPoints.push_back(chessboardCorner);
        }
}


void CameraCalibrator::doCalibration(cv::Mat &cameraMatrix, cv::Mat &dist)
{
    setFilename();
    addPoints();

    std::vector<cv::Mat> rvecs,tvecs;
    cv::calibrateCamera(m_dstpoints,m_srcPoints,m_imageSize,cameraMatrix,dist,rvecs,tvecs);
}
