#include <stdio.h>
#include <memory.h>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "motor_control.hpp"

#define SERVER_PORT					10200		   // 通信ポート
#define DEF_IP						"192.168.0.162"  // サーバーIP


void *threadfunc(void *pdata)
{
    MotorControl *pcontroller = (MotorControl *)pdata;
	unsigned short port = SERVER_PORT;
	int sockClient = -1;
	struct sockaddr_in sockAddrInServer;
  

    while(1){
      if(!pcontroller->m_bConnected){
          // 
			memset(&sockAddrInServer, 0, sizeof(sockAddrInServer));
			sockAddrInServer.sin_port = htons(port);
			sockAddrInServer.sin_family = AF_INET;
			sockAddrInServer.sin_addr.s_addr = inet_addr(DEF_IP);

			sockClient = socket(AF_INET, SOCK_STREAM, 0);
			if (sockClient < 0) {
				printf("ERROR:socket\n");
                usleep(100000);
                continue;
            }
     	    if (connect(sockClient, (struct sockaddr*)&sockAddrInServer, sizeof(sockAddrInServer)) != 0) {
				printf("ERROR:connect\n");
    			close(sockClient);
		    	pcontroller->m_bConnected = false;
			    sockClient = -1;
                usleep(100000);
			    continue;
		    }

            pcontroller->m_bConnected = true;
      }
      else {
			char buff[256];
			sprintf(buff, "###,%3d,%3d,%03d,%03d\r", pcontroller->m_r, pcontroller->m_l, 0, 0);
			if (send(sockClient, buff, ((int)strlen(buff) + 0), 0) < 0) {
				// 送信エラー
				break;
			}
			printf("SC:send[%s]\n", buff);

			memset(buff, 0, sizeof(buff));
			int len = recv(sockClient, buff, 7, 0);
			if (len <= 0) {
    			close(sockClient);
		    	pcontroller->m_bConnected = false;
			    sockClient = -1;
				break;
			}
      }
      usleep(10000);
    }

    return NULL;
}


MotorControl::MotorControl(void)
{
    pthread_t th;

    pthread_create(&th, NULL, threadfunc, this);
}

void MotorControl::drive(int rval, int lval)
{
	printf("JOY STICK MESSAGE %d %d\n", rval, lval);
    m_r = rval;
    m_l = lval;
}

