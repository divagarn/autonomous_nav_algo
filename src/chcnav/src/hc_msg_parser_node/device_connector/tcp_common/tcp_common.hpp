#ifndef __HC_TCP_COMMON_HPP_
#define __HC_TCP_COMMON_HPP_

#include "device_connector.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define TCP_MAX_CONNECT_NUM 300

class tcp_common : public hc__device_connector
{
private:
    int status = -1;                // 连接状态。 -1 未连接，1 已连接
    bool server_mode = false;       // 是否为 TCP Server 模式
    std::string host;               // ip
    int port;                       // 端口号
    int socketfd = -1;              // 已接受客户端 socket fd
    int listenfd = -1;              // 监听 socket fd
    struct sockaddr_in client_addr; // 客户端信息结构体 / 目标地址

    void close_client(void)
    {
    	   if (this->socketfd != -1)
    	   {
    	       close(this->socketfd);
    	       this->socketfd = -1;
    	   }
    	   this->status = -1;
    }

    bool init_listen_socket()
    {
           if (this->listenfd != -1)
               return true;

           this->listenfd = socket(AF_INET, SOCK_STREAM, 0);
           if (this->listenfd < 0)
           {
               fprintf(stderr, "tcp server socket create failed!\n");
               return false;
           }

           int enable = 1;
           setsockopt(this->listenfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

           if (bind(this->listenfd, (struct sockaddr *)&this->client_addr, sizeof(this->client_addr)) < 0)
           {
               fprintf(stderr, "tcp server bind fail!\n");
               close(this->listenfd);
               this->listenfd = -1;
               return false;
           }

           if (listen(this->listenfd, TCP_MAX_CONNECT_NUM) < 0)
           {
               fprintf(stderr, "tcp server listen fail!\n");
               close(this->listenfd);
               this->listenfd = -1;
               return false;
           }

           int flag = fcntl(this->listenfd, F_GETFL, 0);
           fcntl(this->listenfd, F_SETFL, flag | O_NONBLOCK);
           fprintf(stdout, "tcp server listening on port %d\n", this->port);
           return true;
    }

    int accept_client(void)
    {
           if (!init_listen_socket())
               return -1;

           fd_set rfds;
           FD_ZERO(&rfds);
           FD_SET(this->listenfd, &rfds);
           struct timeval tv;
           tv.tv_sec = 0;
           tv.tv_usec = 100 * 1000; // 100 ms

           int ret = select(this->listenfd + 1, &rfds, NULL, NULL, &tv);
           if (ret <= 0)
               return -1;

           int client_fd;
           do
           {
               client_fd = ::accept(this->listenfd, NULL, NULL);
           }
           while (client_fd < 0 && errno == EINTR);

           if (client_fd < 0)
           {
               if (errno == EWOULDBLOCK || errno == EAGAIN)
                   return -1;

               fprintf(stderr, "tcp server accept fail, errno=%d\n", errno);
               return -1;
           }

           this->socketfd = client_fd;
           this->status = 1;
           int flag = fcntl(this->socketfd, F_GETFL, 0);
           fcntl(this->socketfd, F_SETFL, flag | O_NONBLOCK);
           fprintf(stdout, "tcp server accept client success!\n");
           return 0;
    }

public:
    tcp_common(std::string host, int port, bool server = false) : hc__device_connector()
    {
    	   this->host = host;
    	   this->port = port;
    	   this->server_mode = server;
    	   this->socketfd = -1;
    	   this->listenfd = -1;

    	   memset(&this->client_addr, 0, sizeof(this->client_addr));
    	   this->client_addr.sin_family = AF_INET;
    	   this->client_addr.sin_port = htons(this->port);
    	   if (!this->host.empty())
    	       this->client_addr.sin_addr.s_addr = inet_addr(this->host.c_str());
    	   else
    	       this->client_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    }

    int connect(void)
    {
    	   if (this->status != -1 && !this->server_mode)
    	       this->disconnect();

    	   if (!this->server_mode)
    	   {
    	       if (this->socketfd != -1)
    	    	   close(this->socketfd);

    	       this->socketfd = socket(AF_INET, SOCK_STREAM, 0);
    	       if (this->socketfd < 0)
    	       {
    	    	   this->status = -1;
    	    	   fprintf(stderr, "tcp socket create failed!\n");
    	    	   return -1;
    	       }

    	       int ret = ::connect(this->socketfd, (struct sockaddr *)&this->client_addr, sizeof(struct sockaddr_in));
    	       if (ret < 0)
    	       {
    	    	   this->status = -1;
    	    	   fprintf(stderr, "tcp connect fail!\n");
    	    	   close(this->socketfd);
    	    	   this->socketfd = -1;
    	    	   return -1;
    	       }

    	       fprintf(stdout, "tcp connect success!\n");
    	       this->status = 1;

    	       int flag = fcntl(this->socketfd, F_GETFL, 0);
    	       fcntl(this->socketfd, F_SETFL, flag | O_NONBLOCK);

    	       return 0;
    	   }

    	   // Server mode
	   if (this->accept_client() == 0)
	   {
	       return 0;
	   }

	   return -1;
    }

    void judge_connect_state()
    {
           if (this->status == 1)
               return;

           int reconnect_num = 0;
           while (reconnect_num < TCP_MAX_CONNECT_NUM && this->status != 1)
           {
               if (this->connect() == 0)
               {
                   if (this->server_mode)
                       fprintf(stdout, "tcp server connection established\n");
                   else
                       fprintf(stdout, "tcp connection established\n");
                   break;
               }

               reconnect_num++;
               if (this->server_mode)
                   fprintf(stderr, "tcp server waiting for client...\n");
               else
                   fprintf(stderr, "tcp reconnect fail!\n");

               usleep(100 * 1000);
           }
    }

    int write(char *data, unsigned int len)
    {
    	   if (this->status != 1)
    	   {
    	       if (!this->server_mode)
    	       {
    	    	   while (this->status != 1)
    	    	   {
    	    	       fprintf(stderr, "tcp disconnect, reconnect!\n");
    	    	       this->connect();
    	    	       sleep(1);
    	    	   }
    	       }
    	       else
    	       {
    	    	   fprintf(stderr, "tcp server no client connected\n");
    	    	   return -1;
    	       }
    	   }

    	   int write_bytes = ::write(this->socketfd, data, len);
    	   if (write_bytes == -1)
    	   {
    	       printf("socket [%d] send failed\n", this->socketfd);
    	       if (!this->server_mode)
    	    	   this->disconnect();
    	       else
    	    	   close_client();
    	       return -1;
    	   }

    	   return write_bytes;
    }

    int read(char *data, unsigned int maxsize)
    {
    	   if (this->status != 1)
    	   {
    	       if (this->server_mode)
    	    	   return -1;

    	       while (this->status != 1)
    	       {
    	    	   fprintf(stderr, "tcp disconnect, reconnect!\n");
    	    	   this->connect();
    	    	   sleep(1);
    	       }
    	   }

    	   int read_bytes = ::read(this->socketfd, data, maxsize);
    	   if (read_bytes <= 0)
    	   {
    	       if (errno == EWOULDBLOCK || read_bytes == 0)
    	       {
    	    	   if (read_bytes == 0)
    	    	       close_client();
    	    	   errno = 0;
    	    	   return 0;
    	       }
    	       else
    	       {
    	    	   printf("socket [%d] read failed\n", this->socketfd);
    	    	   close_client();
    	    	   return -1;
    	       }
    	   }

    	   return read_bytes;
    }

    int disconnect(void)
    {
    	   if (this->server_mode)
    	   {
    	       close_client();
    	       if (this->listenfd != -1)
    	       {
    	    	   close(this->listenfd);
    	    	   this->listenfd = -1;
    	       }
    	   }
    	   else if (this->socketfd != -1)
    	   {
    	       close(this->socketfd);
    	       this->socketfd = -1;
    	       this->status = -1;
    	   }

    	   return 0;
    }
};

#endif
