#include "Communication.h"
#include <iostream>
#include <vector>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <utility>

Client::Client()
    : mcHostname("localhost"), mcPort(7200), mcMaskClsSize(100)
{
    mImMemSize = -1;
    setupSocket();
    connectSocket();
}

Client::Client(std::string hostname, int port)
    :
    mcHostname(hostname), mcPort(port), mcMaskClsSize(100)
{
    mImMemSize = -1;
    setupSocket();
    connectSocket();
}

Client::~Client()
{
    closeSocket();
}

void Client::sendImages(const std::vector<cv::Mat> &images)
{
    sendImgHeader(images);
    sendImgMat(images);
}

void Client::sendImgHeader(const std::vector<cv::Mat> &images)
{
    cv::Mat image = images[0];
    mImHeight = image.rows;
    mImWidth = image.cols;
    mImChannel = image.channels();
    mImMemSize = image.total() * image.elemSize();
    mImType = image.depth();
    mNumImages = images.size();
    int header[6] = {mNumImages, mImWidth, mImHeight, mImChannel, mImMemSize, mImType};
    bool sen = sendAll(mSockfd, header, sizeof(header));
    if (!sen)
        error("ERROR Sending Header");
}

void Client::sendImgMat(const std::vector<cv::Mat> &images)
{
    int imagesSize = images.size();
    for (int i = 0; i < imagesSize; i++) {
        bool sen = sendAll(mSockfd, images[i].data, mImMemSize);
        if (!sen)
            error("ERROR Sending Images");
//        std::cout << "Image " << i << " Sent ..." << std::endl;
    }
}

bool Client::recvAll(int socket, void *buffer, int length)
{
    uchar *ptr = (uchar *) buffer;
    int bytes = 0;
    while (length > 0) {
        bytes = recv(socket, ptr, length, 0);
        if (bytes < 1)
        {
            errno = ECOMM;
            return false;
        }
        ptr += bytes;
        length -= bytes;
    }
    return true;
}

bool Client::sendAll(int socket, void *buffer, int length)
{
    uchar *ptr = (uchar *) buffer;
    int bytes = 0;
    while (length > 0) {
        bytes = send(socket, ptr, length, 0);
        if (bytes < 1)
            return false;
        ptr += bytes;
        length -= bytes;
    }
    return true;
}

void Client::getSegResult(ClsPosPairs &pairs)
{
    pairs.clear();
    int header[1];
    bool rev_ok = recvAll(mSockfd, header, sizeof(header));
    if (!rev_ok)
        error("ERROR Receiving Header of Segmentation Result");
    int numObjects = header[0];
    for (int i = 0; i < numObjects; i++) {
        int clsNameLen[1];
        rev_ok = recvAll(mSockfd, clsNameLen, sizeof(clsNameLen));
        if (!rev_ok)
            error("ERROR Receiving Object's Name Length");
        uchar className[clsNameLen[0]];
        rev_ok = recvAll(mSockfd, className, clsNameLen[0]);
        if (!rev_ok)
            error("ERROR Receiving Object's Name");
        int clsNum[1];
        rev_ok = recvAll(mSockfd, clsNum, sizeof(clsNum));
        if (!rev_ok)
            error("ERROR Receiving Object's Total Number");
        std::vector<std::vector<double> > poses;
        for (int j = 0; j < clsNum[0]; j++) {
            double pos[3];
            rev_ok = recvAll(mSockfd, pos, sizeof(pos));
            if (!rev_ok)
                error("ERROR Receiving Object's Position");
            std::vector<double> vPos(pos, pos + sizeof(pos) / sizeof(pos[0]));
            poses.push_back(vPos);
        }
        std::string sClassName(className, className + sizeof(className) / sizeof(className[0]));
        std::pair<std::string, std::vector<std::vector<double> > > clsPos(sClassName, poses);
        pairs.push_back(clsPos);
    }
}

void Client::closeSocket()
{
    close(mSockfd);
}

void Client::setupSocket()
{
    int option = 1;
    mSockfd = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(mSockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
    if (mSockfd < 0)
        error("ERROR opening socket");

    mServer = gethostbyname(mcHostname.c_str());
    if (mServer == NULL) {
        fprintf(stderr, "ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &mServAddr, sizeof(mServAddr));
    mServAddr.sin_family = AF_INET;
    bcopy((char *) mServer->h_addr,
          (char *) &mServAddr.sin_addr.s_addr,
          mServer->h_length);
    mServAddr.sin_port = htons(mcPort);
}

void Client::connectSocket()
{
    if (connect(mSockfd, (struct sockaddr *) &mServAddr, sizeof(mServAddr)) < 0)
        error("ERROR connecting");
}

void Client::error(const char *msg)
{
    perror(msg);
    exit(0);
}
