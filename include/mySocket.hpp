//
// Created by hang on 11/2/17.
//

#ifndef CLIENTSERVER_MYSOCKET_H
#define CLIENTSERVER_MYSOCKET_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/wait.h>
#include <signal.h>


class mySocket {

private:
    int connSock;
    int bindSock;
public:

    mySocket();
    ~mySocket();
    void Send(const char *content, unsigned long size);

    int Receive(char *buf, int size);
    int ReceiveAll(char *buf, int size);

    void Close();

    int Bind(const char *port);

    void Listen();

    void Accept();

    int Connect(const char *ip, const char *port);

//    void AcceptAndReply();
};


#endif //CLIENTSERVER_MYSOCKET_H
