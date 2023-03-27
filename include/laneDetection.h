#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <Eigen/Dense>
#include "math.h"

using namespace cv;
using namespace std;
using namespace Eigen;
#define debug(x) cout<<#x<<": "<<x<<endl
#define tdebug(x,y) cout<<#x<<": "<<x<<" | "#y<<": "<<y<<endl

class laneDetection
{
private:
    Mat perspectiveMatrix;
    Mat oriImage;
    Mat edgeImage;
    Mat warpEdgeImage;
    Mat warpOriImage;
    vector<Mat> imageChannels;
    Mat RedBinary;
    Mat mergeImage;
    Mat mergeImageRGB;
    Mat histImage;
    Mat maskImage;
    Mat maskImageWarp;
    Mat finalResult;
    vector<int> histogram; //histograma
    vector<Point2f> laneL;
    vector<Point2f> laneR;
    vector<Point2f> curvePointsL;
    vector<Point2f> curvePointsR;
    int laneLcount;
    int laneRcount;
    int midPoint; //mid.
    int midHeight;
    int leftLanePos; //limite carril
    int rightLanePos; //limite carril
    short initRecordCount; // registrar los 5 primeros frames
    const int blockNum; //numero de blockes
    int stepY; //Window moving step.
    const int windowSize; //tamaño slide windows
    Vector3d curveCoefL; //coeficiente de curva derecha
    Vector3d curveCoefR; //ciefucionete curva izquierda).
    Vector3d curveCoefRecordL[5]; //ultimos 5 registros
    Vector3d curveCoefRecordR[5]; //
    int recordCounter;
    bool failDetectFlag; // indicar si las marcas de carreteras se detectan 1
    void calHist();
    void boundaryDetection();
    void laneSearch(const int &lanePos, vector<Point2f> &_line, int &lanecount, vector<Point2f> &curvePoints, char dir);
    bool laneCoefEstimate();
    void laneFitting();

public:
    int errorframe;
    int count();
    laneDetection(Mat _oriImage, Mat _perspectiveMatrix);
    ~laneDetection();
    void laneDetctAlgo(string);
    Mat getEdgeDetectResult();
    Mat getWarpEdgeDetectResult();
    Mat getRedChannel();
    Mat getRedBinary();
    Mat getMergeImage();
    Mat getHistImage();
    Mat getMaskImage();
    Mat getWarpMask();
    Mat getFinalResult();
    float getLaneCenterDist();
    void setInputImage(Mat &image);
};



laneDetection::laneDetection(const Mat _oriImage, const Mat _perspectiveMatrix):oriImage(_oriImage), perspectiveMatrix(_perspectiveMatrix), blockNum(9), windowSize(150), recordCounter(0), initRecordCount(0), failDetectFlag(true)
{
    histogram.resize(_oriImage.size().width);
    midPoint = _oriImage.size().width >> 1;
    midHeight = _oriImage.size().height * 0.55;
    stepY = oriImage.size().height / blockNum;
    Vector3d initV;
    initV << 0, 0, 0;
    for(int i=0; i<5; i++)
    {
        curveCoefRecordL[i] = initV;
    }
    errorframe = 0;
}

laneDetection::~laneDetection() {}
//pipe
void laneDetection::laneDetctAlgo(string name)
{

    int lowH = 0; int highH = 26;
    int lowS = 98; int highS = 118;
    int lowV = 71;  int highV = 183;

    //canny
    Mat oriImageGray;
    //imshow("recibido",oriImage);
    cvtColor(oriImage, oriImageGray, COLOR_RGB2GRAY);


    //Canny(oriImageGray, edgeImage, 100, 150, 3);
    Mat imgHSV;

    cvtColor(oriImage, imgHSV, COLOR_BGR2HSV);


    inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), edgeImage);
    //imshow("imagethres",edgeImage);
    warpPerspective(edgeImage, warpEdgeImage, perspectiveMatrix, edgeImage.size());


    //    cout << "Width : " << edgeImage.size().width << endl;
    //    cout << "Height: " << edgeImage.size().height << endl;

    //    imshow("warpEdge",warpEdgeImage);





    inRange(warpEdgeImage, Scalar(1),Scalar(255),warpEdgeImage);
    //Split the color image into different channels.
    warpPerspective(oriImage, warpOriImage, perspectiveMatrix, oriImage.size());
    split(warpOriImage, imageChannels);



    //cannal rojo
    inRange(imageChannels[2], Scalar(200), Scalar(255),RedBinary);
    //Merge the binarized R channel image with edge detected image.
    add(warpEdgeImage, RedBinary, mergeImage);
    cvtColor(mergeImage, mergeImageRGB, COLOR_GRAY2RGB);

    //histogram.
    calHist();

    //umbral
    boundaryDetection();

    //buscar  curva
    laneSearch(leftLanePos, laneL, laneLcount, curvePointsL, 'L');
    laneSearch(rightLanePos, laneR, laneRcount, curvePointsR, 'R');
    laneCoefEstimate();
    laneFitting();
    warpPerspective(maskImage, maskImageWarp, perspectiveMatrix, maskImage.size(),WARP_INVERSE_MAP);
}


//frecuencia con las que aparecen los distintos niveles
void laneDetection::calHist()
{
    histogram.clear();
    for(int i = 0; i < mergeImage.size().width; i++)
    {
        Mat ROI = mergeImage(Rect(i, oriImage.size().height-midHeight-1, 1, midHeight));
        Mat dst;
        divide(255, ROI, dst);
        histogram.push_back((int)(sum(dst)[0]));
    }
    int maxValue = 0;
    maxValue = (*max_element(histogram.begin(), histogram.end())); //the maximum value of the histogram.
    histImage.create(maxValue, histogram.size(), CV_8UC3);
    histImage = Scalar(255,255,255);

    //To create the histogram image
    for(int i=0; i<histogram.size(); i++)
    {
        line(histImage, Point2f(i,(maxValue-histogram.at(i))), Point2f(i,maxValue), Scalar(0,0,255), 1);
        //imshow("--->",histImage);
    }
}

//limites
void laneDetection::boundaryDetection()
{
    //izquierda
    vector<int>::iterator maxLPtr;
    maxLPtr = max_element(histogram.begin(), histogram.begin()+midPoint-1);
    int maxL = *maxLPtr;

    leftLanePos = distance(histogram.begin(),maxLPtr);
    //tdebug(maxL,leftLanePos);


    //derecha
    vector<int>::iterator maxRPtr;
    maxRPtr = max_element(histogram.begin()+midPoint, histogram.end());
    int maxR = *maxRPtr;
    rightLanePos = distance(histogram.begin(),maxRPtr);
    //tdebug(maxR,rightLanePos);

    //draw
    if((initRecordCount < 5) || (failDetectFlag == true))
    {
        line(mergeImageRGB, Point2f(leftLanePos, 0), Point2f(leftLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);
        line(mergeImageRGB, Point2f(rightLanePos, 0), Point2f(rightLanePos, mergeImageRGB.size().height), Scalar(0, 0, 255), 10);
    }
}


//Curva
void laneDetection::laneSearch(const int &lanePos, vector<Point2f> &_line, int &lanecount, vector<Point2f> &curvePoints, char dir)
{
    _line.clear();


    const int skipStep = 4;//pass
    int nextPosX = lanePos;
    int xLU = 0, yLU = 0;
    int xRB = 0, yRB = 0;
    int _windowSize = windowSize;
    //int _stepY = stepY;
    int sumX = 0;
    int xcounter = 0;
    lanecount = 0;


    if((initRecordCount < 5) || (failDetectFlag == true)) //busqueda completa
    {
        for(int i=0; i<blockNum; i++)
        {
            _windowSize = windowSize;
            xLU = nextPosX - (windowSize >> 1); //(x) del punto superior izquierdo
            yLU = stepY*(blockNum-i -1); // (y) del punto superior izquierdo
            //tdebug(xLU,yLU);

            xRB = xLU + windowSize; // x punto inferior derecho
            yRB = yLU + stepY -1; /// y punto inferior derecho
            //tdebug(xLU,yLU);

            // si detecta carril
            if((xLU < 0))
            {
                xLU =0;
                xRB = xLU + windowSize;
            }
            if(xRB > (mergeImage.size().width-1))
            {
                _windowSize = windowSize + ((mergeImage.size().width-1) - xRB);
                xRB = (mergeImage.size().width-1);
                xLU += ((mergeImage.size().width-1) - xRB);
            }
            if(xRB-xLU > 0 && xRB >= 0 && xLU >= 0)
            {
                //Detectar puntos en las ventanas
                sumX = 0;
                xcounter = 0;
                uchar* matPtr;//valor pixel
                for(int j=yLU; j<=yRB; j+=skipStep)
                {
                    matPtr = mergeImage.data + (j*mergeImage.size().width);
                    for(int k=xLU; k<=xRB; k+=skipStep)
                    {
                        if(*(matPtr+k) == 255)
                        {
                            sumX += k; xcounter++;
                        }
                    }
                }
                if (xcounter!=0) sumX /= xcounter; //media de los puntos encontrados
                else sumX = nextPosX;

                //modificar posion de la ventana en base a la media
                nextPosX = sumX;
                xLU = ((nextPosX-(windowSize>>1))>0)? (nextPosX-(windowSize>>1)) : 0;
                xRB = ((xLU + windowSize) < (mergeImage.size().width))? (xLU + windowSize) : (mergeImage.size().width-1);
                if(xRB-xLU > 0 && xRB >= 0 && xLU >= 0)
                {
                    for(int j=yLU; j<=yRB; j+=skipStep)
                    {
                        matPtr = mergeImage.data + (j*mergeImage.size().width);
                        for(int k=xLU; k<=xRB; k+=skipStep)
                        {
                            if(*(matPtr+k) == 255)
                            {
                                lanecount++;
                                _line.push_back(Point2f(k,j));
                            }
                        }
                    }
                }
                rectangle(mergeImageRGB, Point2f(xLU, yLU), Point2f(xRB, yRB),Scalar(255, 0, 0), 5);
            }

        }
    }
    else //usando resultados previos
    {
        uchar* matPtr;
        int xtemp;
        for(int i=0; i<mergeImage.size().height; i++)
        {
            matPtr = mergeImage.data + (i*mergeImage.size().width);
            for(int j=-50; j<=50; j+=3 )
            {
                xtemp = (curvePoints[i].x + j);
                if(xtemp>=0 && xtemp<mergeImage.size().width)
                {
                    if(*(matPtr+xtemp) == 255)
                    {
                        lanecount++;
                        _line.push_back(Point2f(xtemp,i));
                        if(i>=(mergeImage.size().height/2))
                        {
                            sumX += xtemp;
                            xcounter++;
                        }

                    }
                    mergeImageRGB.at<Vec3b>(i,xtemp)[0] = 0;
                    mergeImageRGB.at<Vec3b>(i,xtemp)[1] = 255;
                    mergeImageRGB.at<Vec3b>(i,xtemp)[2] = 255;
                }
            }
        }
        sumX /= xcounter;
        if((sumX > 0) && (sumX < mergeImageRGB.size().width))
        {
            if(dir == 'L')
            {
                leftLanePos = sumX;
                line(mergeImageRGB, Point2f(leftLanePos, 0), Point2f(leftLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);

            }
            else
            {
                rightLanePos = sumX;
                line(mergeImageRGB, Point2f(rightLanePos, 0), Point2f(rightLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);
            }
        }
        else
        {
            if(dir == 'L') line(mergeImageRGB, Point2f(leftLanePos, 0), Point2f(leftLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);

            else line(mergeImageRGB, Point2f(rightLanePos, 0), Point2f(rightLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);
        }
    }

}



bool laneDetection::laneCoefEstimate()
{
    //minimos cuadrados
    int countThreshold = 300;
    if((laneLcount > countThreshold) && (laneRcount > countThreshold))
    {
        VectorXd xValueL(laneLcount);
        VectorXd xValueR(laneRcount);
        MatrixXd leftMatrix(laneLcount,3);
        MatrixXd rightMatrix(laneRcount,3);

        //estimar coeficiente de curva
        for(int i=0; i<laneLcount; i++)
        {
            xValueL(i) = laneL[i].x;
            leftMatrix(i,0) = pow(laneL[i].y, 2);
            leftMatrix(i,1) = laneL[i].y;
            leftMatrix(i,2) = 1;
        }
        //debug(leftMatrix(i,3));

        for(int i=0; i<laneRcount; i++)
        {
            xValueR(i) = laneR[i].x;
            rightMatrix(i,0) = pow(laneR[i].y, 2);
            rightMatrix(i,1) = laneR[i].y;
            rightMatrix(i,2) = 1;
        }

        //debug(xValueL);

        curveCoefL = (leftMatrix.transpose()*leftMatrix).ldlt().solve(leftMatrix.transpose()*xValueL);
        curveCoefR = (rightMatrix.transpose()*rightMatrix).ldlt().solve(rightMatrix.transpose()*xValueR);

        curveCoefRecordL[recordCounter] = curveCoefL;
        curveCoefRecordR[recordCounter] = curveCoefR;
        recordCounter = (recordCounter + 1) % 5;
        if(initRecordCount<5) initRecordCount++;
        failDetectFlag = false;
        return true;
    }
    else
    {
        cerr << "[ERROR LINE] mas de una linea";
        errorframe+=1;
        cout<<errorframe<<endl;
        failDetectFlag = true;
        return false;
    }
}


//ajustar linea
void laneDetection::laneFitting()
{
    maskImage.create(mergeImage.size().height, mergeImage.size().width, CV_8UC3);
    maskImage = Scalar(0,0,0);
    curvePointsL.clear();
    curvePointsR.clear();

    //5 frames
    if(initRecordCount == 5)
    {
        curveCoefL = (curveCoefRecordL[0] + curveCoefRecordL[1] + curveCoefRecordL[2] + curveCoefRecordL[3] + curveCoefRecordL[4]) / 5;
        curveCoefR = (curveCoefRecordR[0] + curveCoefRecordR[1] + curveCoefRecordR[2] + curveCoefRecordR[3] + curveCoefRecordR[4]) / 5;
    }

    int xL, xR;
    for(int i=0; i<mergeImage.size().height; i++)
    {
        xL= pow(i,2) * curveCoefL(0) + i * curveCoefL(1) + curveCoefL(2);
        xR= pow(i,2) * curveCoefR(0) + i * curveCoefR(1) + curveCoefR(2);
        if(xL < 0) xL=0;
        if(xL >= mergeImage.size().width) xL = mergeImage.size().width -1;
        if(xR < 0) xR=0;
        if(xR >= mergeImage.size().width) xR = mergeImage.size().width -1;
        curvePointsL.push_back(Point2f(xL,i));
        curvePointsR.push_back(Point2f(xR,i));
    }
    Mat curveL(curvePointsL, true);
    curveL.convertTo(curveL, CV_32S);
    polylines(maskImage, curveL, false, Scalar(255,0,0), 20, LINE_AA);
    Mat curveR(curvePointsR, true);
    curveR.convertTo(curveR, CV_32S);
    polylines(maskImage, curveR, false, Scalar(0,0,255), 20, LINE_AA);

    uchar* matPtr;
    for(int i=0; i<maskImage.size().height; i++)
    {
        matPtr = maskImage.data + i * maskImage.size().width * 3;
        for(int j = curvePointsL[i].x; j <= curvePointsR[i].x; j++)
        {
            *(matPtr + j*3) = 2;
            *(matPtr + j*3 + 1) = 25;
            *(matPtr + j*3 + 2) = 51;
        }
    }

}

Mat laneDetection::getEdgeDetectResult()
{
    return edgeImage;
}


Mat laneDetection::getWarpEdgeDetectResult()
{
    return warpEdgeImage;
}

Mat laneDetection::getRedChannel()
{
    return imageChannels[2];
}

Mat laneDetection::getRedBinary()
{
    return RedBinary;
}

Mat laneDetection::getMergeImage()
{
    return mergeImageRGB;
}

Mat laneDetection::getHistImage()
{
    return histImage;
}

Mat laneDetection::getMaskImage()
{
    return maskImage;
}

Mat laneDetection::getWarpMask()
{
    return maskImageWarp;
}

Mat laneDetection::getFinalResult()
{
    addWeighted(maskImageWarp, 0.5, oriImage, 1, 0, finalResult);
    return finalResult;
}

void laneDetection::setInputImage(Mat &image)
{
    oriImage = image.clone();
}

float laneDetection::getLaneCenterDist()
{
    float laneCenter = ((rightLanePos - leftLanePos) / 2) + leftLanePos;
    float imageCenter = mergeImageRGB.size().width / 2;
    debug(imageCenter);

    cv::circle(mergeImageRGB,Point2f(laneCenter,imageCenter), 6, Scalar(0,255,0), FILLED);


    //imshow("goo",mergeImageRGB);
    float result;
    //Assume the lane width is 3.5m and about 600 pixels in our image.
    result = (laneCenter -imageCenter)* 3.5 / 600;
    cv::circle(mergeImageRGB,Point2f(laneCenter,result), 6, Scalar(0,0,255), FILLED);

    line(mergeImageRGB, Point2f(laneCenter,imageCenter+result), Point2f(laneCenter,result), Scalar(255, 0, 0),
         3, LINE_8);
    //debug(result);
    //imshow("goo",mergeImageRGB);
    return result;
}

int laneDetection::count(){
    return errorframe;
}
