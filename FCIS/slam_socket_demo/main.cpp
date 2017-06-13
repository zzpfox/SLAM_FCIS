#include <iostream>
#include "Communication.h"
#include <chrono>
#include <ctime>
#include <Eigen/Dense>
#include <unordered_map>

#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

struct ObjectPos{
    std::vector<Eigen::Vector3d> Pcs;
    Eigen::Matrix4d Tcw;
    ObjectPos(){}
    ObjectPos(std::vector<Eigen::Vector3d> pcs, Eigen::Matrix4d tcw): Pcs(pcs), Tcw(tcw){}
    ObjectPos(Eigen::Matrix4d tcw): Tcw(tcw){}

    void addInstance(Eigen::Vector3d& Pc)
    {
        Pcs.push_back(Pc);
    }

    void updateKeyFramePos(Eigen::Matrix4d t)
    {
        Tcw = t;
    }

    friend std::ostream& operator << (std::ostream &os, const ObjectPos& pos)
    {

        os << std::setw(6) << std::fixed << std::setprecision(3);
        os << "--------------------------\n" << "Pcs: " << std::endl;
        for (auto& Pc: pos.Pcs)
        {
            os << "[" << Pc[0] << " " << Pc[1] << " " << Pc[2] << "]" << std::endl;
        }
        os << "Tcw: " << "\n" << pos.Tcw << "\n";
        os << "--------------------------" << std::endl;
        return os;
    }
};


int send(Client& client,
         std::unordered_map<std::string, std::unordered_map<long unsigned int, ObjectPos> >& objectMap,
         long unsigned int keyFrameId)
{
    cv::Mat color_img = cv::imread("ImgColor2.png", cv::IMREAD_COLOR);
    if (!color_img.data)
    {
        std::cout <<  "Could not open or find the color image" << std::endl ;
        return -1;
    }
    std::vector<cv::Mat> color_imgs;
    color_imgs.push_back(color_img);

    cv::Mat depth_img = cv::imread("ImgDepth2.png", cv::IMREAD_ANYDEPTH);
    if (!depth_img.data)
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    depth_img.convertTo(depth_img,CV_32F,1/1000.0);

//    double min, max;
//    cv::Point minLoc;
//    cv::Point maxLoc;
//    cv::Mat cameraFeedFlat = depth_img.reshape(1);
//    cv::minMaxLoc(cameraFeedFlat, &min, &max, &minLoc, &maxLoc );
//    std::cout << "Min: " << min << " Max: " << max << std::endl;

    std::vector<cv::Mat> depth_imgs;
    depth_imgs.push_back(depth_img);

    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    start = std::chrono::system_clock::now();
    std::cout<< std::endl << "==================================="<<std::endl;
    client.sendImages(color_imgs);
    client.sendImages(depth_imgs);

    Client::ClsPosPairs clsPosPairs;
    std::cout<< "Sending Done"<<std::endl;
    client.getSegResult(clsPosPairs);

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << FGRN("Sending and Receiving Elapsed time: ") << elapsed_seconds.count() << "s\n";


    std::cout<< FBLU("--- Response ---")<<std::endl;

    for (int i = 0; i < clsPosPairs.size(); i++)
    {
        std::string objname = clsPosPairs[i].first;
        std::cout<< "Name: " << objname << std::endl;
        std::cout<< "Position: " << std::setw(6) << std::fixed << std::setprecision(3) << std::endl;
        std::vector<std::vector<double> > poses = clsPosPairs[i].second;

        std::unordered_map<long unsigned int, ObjectPos> objKeyFrameMap;
        ObjectPos objpos;
        Eigen::Matrix4d Tcw;
        Tcw << Eigen::Matrix4d::Ones();
        objpos.Tcw = Tcw;
        for (int j = 0; j < poses.size(); j++)
        {
            std::vector<double> pos = poses[j];
            std::cout<< "          ";
            for (int k = 0; k < pos.size(); k++)
            {
                std::cout << pos[k] << " ";
            }
            std::cout << std::endl;
            Eigen::Vector3d Pc(pos.data());
            objpos.addInstance(Pc);
        }
        std::cout << std::endl;

        objKeyFrameMap[keyFrameId] = objpos;
        auto it = objectMap.find(objname);
        if (it != objectMap.end())
        {
            it -> second.insert(objKeyFrameMap.begin(), objKeyFrameMap.end());
        }
        else
        {
            objectMap.insert(std::make_pair(objname, objKeyFrameMap));
        }
    }
    std::cout << FMAG("Printing the Hash Maps") << std::endl;
    for (auto& x:objectMap){
        std::cout << KCYN << x.first << RST << ": " << std::endl;
        for (auto& y: x.second){
            std::cout << "KeyFrame " << y.first << ": " << std::endl;
            std::cout << y.second << std::endl;
        }
    }

    return 0;
}
int main() {
    Client client;
    std::unordered_map<std::string, std::unordered_map<long unsigned int, ObjectPos> > objectMap;
    for (int i = 0; i < 3; i++)
    {
        send(client, objectMap, i);
    }

    return 0;
}