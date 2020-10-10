//////////////////////////////////////////////////////////////////////////
//UDP broadcast receiver for Oxts IMU
//Author: John Zhao
//Date: 2015.6.1
//////////////////////////////////////////////////////////////////////////
#pragma once
//#include "VirtualSwitchBus.h"

#include <stdlib.h>
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "NComRxC.h"
#include "stdio.h"

//////////////////////////////////////////////////////////////////////////
//TODO:dynamicly load virtualswitchbus
////wrapper classes for SOCKET by Peter R.
//class WSASession
//{
//private:
//	WSAData data;
//public:
//	WSASession()
//	{
//		int ret = WSAStartup(MAKEWORD(2, 2), &data);
//		if (ret != 0)
//			fprintf(stdout, "WSAStartup Failed\n");
//	}
//	~WSASession()
//	{
//		WSACleanup();
//	}
//};
void error(const char *msg)
{
    perror(msg);
    exit(0);
}

class UDPSocket
{
private:
//    SOCKET sock;
    int sock;
public:
	UDPSocket()
	{
		sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
//		BOOL bOptVal = true;
		bool bOptVal = true;
//        int bOptLen = sizeof(BOOL);
        int bOptLen = sizeof(bOptVal);
		setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&bOptVal, bOptLen);
//        if (sock == INVALID_SOCKET)
        if (sock < 0)
		{
			printf("Error opening socket. \n");
		}
	}
	~UDPSocket()
	{
//        closesocket(sock);
        close(sock);
	}
	void Bind(const short port, const char* ip, bool bBindIp)
	{
		sockaddr_in add;
		add.sin_family = AF_INET;
		if (bBindIp)
		{
			add.sin_addr.s_addr = inet_addr(ip);
		}
		else
		{
			add.sin_addr.s_addr = htons(INADDR_ANY);
		}
		add.sin_port = htons(port);

//        int ret = bind(sock, reinterpret_cast<SOCKADDR *>(&add), sizeof(add));
        int ret = bind(sock, (struct sockaddr *)(&add), sizeof(add));
		if (ret < 0 )
		{
//			int error = WSAGetLastError();
//      	printf("Error binding socket. \n");
            error("Error binding socket \n");
		}
	}
	sockaddr_in RecvFrom(char* buffer, int len, int flags = 0)
	{
		sockaddr_in from;
        socklen_t  size = sizeof(from);
		int ret = recvfrom(sock, buffer, len, flags,  (struct sockaddr *)(&from), &size);
		if (ret < 0 )
		{
//			int error = WSAGetLastError();
//			printf("Error receiving buffer. \n");
            error("Error receiving buffer. \n");
		}
		//terminate the buffer
		buffer[ret] = '/0';
		return from;
	}
};

//////////////////////////////////////////////////////////////////////////
//the receiver and decoder of NCOMM (provided by the manufacture)
class IMU_RECEIVER
{
private:
	NComRxC *nrx; //NComRxC object
public:
	//constructor
	IMU_RECEIVER()
	{
		//Create NCOM decoder and check
		nrx = NComCreateNComRxC();
		if(nrx == NULL)
		{
			printf("Error: Unable to create NCom decoder.\n");
		}

	}
	//destructor
	~IMU_RECEIVER()
	{
		//Clean up
		NComDestroyNComRxC(nrx);
	}
	//functions
	NComRxC* get_ncom() { return nrx; }
};

////multi thread
//void listen_decode(LPVOID lpParam);
//void report(LPVOID lpParam);//report the buffer