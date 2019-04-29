#include "utils.h"
#include <Eigen/Dense>
#include <iostream>
#include <string> 
#include <fstream>
#include <sstream>
using namespace cv;
using namespace std;

cv::Mat Utils::Quaternion2RotM(const float& x, const float& y, const float& z, const float& w)
{
    float xx = x*x;
    float yy = y*y;
    float zz = z*z;

    float wx = w*x;
    float wy = w*y;
    float wz = w*z;

    float xy = x*y;
    float xz = x*z;
    float yz = y*z;

    Mat R = ( Mat_<float> ( 3,3 ) <<
                ( 1 - 2*yy - 2*zz ), ( 2*xy - 2*wz ), ( 2*xz + 2*wy ),
                ( 2*xy + 2*wz ), ( 1 - 2*xx - 2*zz ), ( 2*yz - 2*wx ),
                ( 2*xz - 2*wy ), ( 2*yz + 2*wx ), ( 1 - 2*xx - 2*yy ));
    return R;
}

void Utils::ReadCameraTimeAndPose(const string& csv_path_pose, const double& timestart, const double& timeend,
                                vector<string>& imgtimes, vector<Mat>& Twcs)
{
    imgtimes.clear();
    Twcs.clear();
   
    ifstream file(csv_path_pose);
    std::string line;
    int row = 0;
    while(std::getline(file, line))
    {
        if(row==0)
        {
            row++;
            continue;       
        }

        stringstream ss(line);
        string str;
        
        int col = 0;
        double timesec, timensec;
        double posx, posy, posz;
        double qx, qy, qz, qw;
        while (getline(ss, str, ','))
        {
            std::stringstream convertor(str);
            double value;
            convertor >> value;

            if(col==2)
                timesec = value;
            else if(col==3)
                timensec = value;
            else if(col==6)
                posx = value;
            else if (col==7)
                posy = value;
            else if (col==8)
                posz = value;
            else if (col==9)
                qx = value;
            else if (col==10)
                qy = value;
            else if (col==11)
                qz = value;
            else if (col==12)
                qw = value;


            col++;
        }
        
        timensec = timensec/1000000000;
        timesec = timesec + timensec;
        string name =  std::to_string(timesec).substr(0,13);
        double named = stod(name);

        if(named>=timestart && named<=timeend)
        {
            imgtimes.push_back(name);

            Mat R = Quaternion2RotM(qx, qy, qz, qw);
            Mat T = (Mat_<float> (4,4) <<
                R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2), posx,
                R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2), posy,
                R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2), posz,
                                0,                0,                0,     1
            );
            Twcs.push_back(T);
        }

        row++;        
    }
}

void Utils::ReadImgNameAndVertices(const string& csv_path_vertice, const vector<string>& imgtimes, 
                                        vector<string>& imgnames, vector<vector<Point2f>>& verticess)
{
    imgnames.clear();
    verticess.clear();
    ifstream file(csv_path_vertice);
    std::string line;
    int row = 0;

    while(std::getline(file, line))
    {
        if(row==0)
        {
            row++;
            continue;       
        }

        stringstream ss(line);
        string str;

        int col = 0;
        string imgname;
        int LTx, LTy, RTx, RTy, RBx, RBy, LBx, LBy;
        while (getline(ss, str, ','))
        {
            if(col==0)
                imgname = str;
            else
            {
                std::stringstream convertor(str);
                double value;
                convertor >> value;

                if(col==1)
                    LTx = value;
                else if (col==2)
                    LTy = value;
                else if (col==3)
                    RTx = value;
                else if (col==4)
                    RTy = value;
                else if (col==5)
                    RBx = value;
                else if (col==6)
                    RBy = value;
                else if (col==7)
                    LBx = value;
                else if (col==8)
                    LBy = value;
            }
            col++;
        }

        double imgnamed = stod(imgname.substr(0,13));

        for(int i=0; i<imgtimes.size(); i++)
        {
            double imgtimed = stod(imgtimes[i]);

            if(abs(imgnamed-imgtimed)<=0.015)
            {
                imgnames.push_back(imgname.substr(0,13)+".png");
                vector<cv::Point2f> vertices;
                vertices.push_back(Point2f(LTx, LTy));
                vertices.push_back(Point2f(RTx, RTy));
                vertices.push_back(Point2f(RBx, RBy));
                vertices.push_back(Point2f(LBx, LBy));
                verticess.push_back(vertices);
            }
        }
        row++;
    }
}
