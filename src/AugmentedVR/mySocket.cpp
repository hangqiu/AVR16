//
// Created by hang on 11/2/17.
//

#include "mySocket.hpp"
#include <iostream>
using namespace std;

mySocket::mySocket(){

}
mySocket::~mySocket(){
    Close();
}


void mySocket::Send(const char* content, unsigned long size){
    if (send(connSock, content, size, 0) == -1) {
        perror("send");
    }
}

int mySocket::Receive(char* buf, int size){
    int numbytes;
    if ((numbytes = recv(connSock, buf, size, 0)) == -1) {
        perror("recv");
    }
    if (numbytes != size){
        perror("received less than speced\n");
        std::cout << numbytes << endl;
    }
    buf[numbytes]='\0';
    return numbytes;
}

int mySocket::ReceiveAll(char* buf, int size){
    int numbytes;
    if ((numbytes = recv(connSock, buf, size, MSG_WAITALL)) == -1) {
        perror("recv");
    }
    if (numbytes != size){
        perror("received less than speced\n");
        std::cout << numbytes << endl;
    }
    buf[numbytes]='\0';
    return numbytes;
}


void mySocket::Close(){
    close(connSock);
    close(bindSock);
}



int mySocket::Bind(const char * port){
    int yes = 1;

    struct addrinfo hints={0};
    struct addrinfo *serverinfo,*p;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    int rv;

    //code reused from Beej's
    if ((rv = getaddrinfo(NULL, port, &hints, &serverinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and bind to the first we can
    for (p = serverinfo; p != NULL; p = p->ai_next) {
        if ((bindSock = socket(p->ai_family, p->ai_socktype, p->ai_protocol))
            == -1) {
            perror("server: socket");
            continue;
        }

        if (setsockopt(bindSock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int))
            == -1) {
            perror("setsockopt buffer size");
            exit(1);
        }

        int size = 6000000;

        if (setsockopt(bindSock, SOL_SOCKET, SO_SNDBUF, &size, sizeof(int))
            == -1) {
            perror("setsockopt");
            exit(1);
        }

        if (bind(bindSock, p->ai_addr, p->ai_addrlen) == -1) {
            close(bindSock);
            perror("server: bind");
            continue;
        }

        break;
    }

    if (p == NULL) {
        fprintf(stderr, "server: failed to bind\n");
        return 2;
    }

    freeaddrinfo(serverinfo); // all done with this structure

}

void mySocket::Listen(){
    if (listen(bindSock, 10) == -1) {
        perror("listen");
        exit(1);
    }
}

int mySocket::Connect(const char* ip, const char* port){
    struct sockaddr_storage my_addr;
    struct addrinfo hints={0};
    struct addrinfo * servinfo,*p;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int rv;
    //code reused from Beej's
    if ((rv = getaddrinfo(ip, port, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and connect to the first we can
    for (p = servinfo; p != NULL; p = p->ai_next) {
        if ((connSock = socket(p->ai_family, p->ai_socktype, p->ai_protocol))
            == -1) {
            perror("client: socket");
            continue;
        }

        if (connect(connSock, p->ai_addr, p->ai_addrlen) == -1) {
            close(connSock);
            perror("client: connect");
            continue;
        }

        break;
    }

    // Retrieve the locally-bound name of the specified socket
    // and store it in the sockaddr structure
    socklen_t addr_len = sizeof my_addr;
    if (getsockname(connSock, (struct sockaddr *) &my_addr, &addr_len) == -1) {
        perror("getsockname");
        exit(1);
    }

    printf("Client has dynamic TCP port number %d\n",(int) ntohs(((struct sockaddr_in*) &my_addr)->sin_port));

    if (p == NULL) {
        fprintf(stderr, "client: failed to connect\n");
        return 2;
    }

    freeaddrinfo(servinfo); // all done with this structure
}

void mySocket::Accept(){
    struct sockaddr_storage their_addr; // connector's address information
    socklen_t sin_size = sizeof their_addr;
    connSock = accept(bindSock, (struct sockaddr *) &their_addr, &sin_size);// listen on sock_fd, new connection on new_fd
//    char s[INET6_ADDRSTRLEN];
//    inet_ntop(their_addr.ss_family,
//              get_in_addr((struct sockaddr *) &their_addr), s, sizeof s);
}



//void mySocket::AcceptAndReply(){
//
//
//    /// accept
//    struct sockaddr_storage their_addr; // connector's address information
//    socklen_t sin_size = sizeof their_addr;
//    connSock = accept(bindSock, (struct sockaddr *) &their_addr, &sin_size);// listen on sock_fd, new connection on new_fd
//    char s[INET6_ADDRSTRLEN];
//    inet_ntop(their_addr.ss_family,
//              get_in_addr((struct sockaddr *) &their_addr), s, sizeof s);
//
//    //code reused from Beej's
//    if (!fork()) { // this is the child process
//        close(bindSock); // child doesn't need the listener
//
//        char buf[MAXBUFLEN];
//        int numbytes = Receive(connSock,buf,MAXBUFLEN);
//        printf("File Server received: %s\n",buf);
//
//        char reply[MAXBUFLEN] = "File Server Reply";
//        Send(connSock,reply,MAXBUFLEN);
//        printf("File Server has replied %s.\n", reply);
//
//        Close(connSock);
//        exit(0);
//    }
//    Close(connSock);  // parent doesn't need this
//}



////code reused from Beej's


// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa) {
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*) sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*) sa)->sin6_addr);
}

void *get_in_port(struct sockaddr *sa) {
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*) sa)->sin_port);
    }

    return &(((struct sockaddr_in6*) sa)->sin6_port);
}

