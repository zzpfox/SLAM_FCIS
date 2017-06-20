#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <netinet/in.h>

class Client
{
public:
    typedef std::vector<std::pair<std::string, std::vector<std::vector<double> > > > ClsPosPairs;

    Client();

    Client(std::string hostname, int port);

    ~Client();

    void sendImages(const std::vector<cv::Mat> &images);

    void getSegResult(ClsPosPairs &pairs);

    void error(const char *msg);

    bool recvAll(int socket, void *buffer, int length);

    bool sendAll(int socket, void *buffer, int length);

private:
    void closeSocket();

    void setupSocket();

    void connectSocket();

    void sendImgHeader(const std::vector<cv::Mat> &images);

    void sendImgMat(const std::vector<cv::Mat> &images);

private:
    const std::string mcHostname;
    const int mcPort;
    const int mcMaskClsSize;
    int mImHeight;
    int mImWidth;
    int mImChannel;
    int mImType;
    int mNumImages;
    int mSockfd;
    int mImMemSize;
    struct hostent *mServer;
    struct sockaddr_in mServAddr;
};

#endif
