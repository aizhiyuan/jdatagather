/**
 * ACR_network.c includes several functions below:
 * int opensock()
 * int req_conn(int sockfd ,char *ip_addr,int port)
 * int recv_data(int sockfd,char *recv_buf,int RECV_MAXDATASIZE)
 * int link_network(char *ip)
 */
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include "CD_network.h"
#include "log.h"

/* ----------------------------------------------------------------------------------- */
/**
 * @brief 创建一个套接字描述符
 *
 * @return int
 */
// open sock
int opensock()
{
    int sockfd;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1)
    {
        // perror("socket error");
        LOG_ERROR("errno number is %d\n", errno);
        return (-1);
    }
    LOG_SUCCESS("open socket success!\n");
    return sockfd;
}

/* ----------------------------------------------------------------------------------- */
/**
 * @brief 请求连接
 * @param sockfd
 * @param ip_addr
 * @param port
 *
 * @return int
 */
// req conn
int req_conn(int sockfd, const char *ip_addr, int port)
{
    /*
        int sockfd;
        int z;
        int numbytes;
        int IPAddrStatus;
        int i;

        char buf[MAXSIZE];
        struct sockaddr_in adr_srvr;
        int addr_port = PORT;
        int len_inet = sizeof(struct sockaddr_in);
        memset(&adr_srvr, 0, len_inet);
        adr_srvr.sin_family = AF_INET;
        adr_srvr.sin_port = htons(addr_port);
        IPAddrStatus = inet_aton("192.168.2.210", &adr_srvr.sin_addr);
        if (IPAddrStatus == 0) {
            bail("inet_aton()");
        }

        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd == -1) {
            bail("socket()");
        }

        z = connect(sockfd, (struct sockaddr *) &adr_srvr, len_inet);
        if (z == -1) {
            bail("connect()");
        }
        else{
            printf("connct success\n");
        }

        sleep(60);*/

    struct sockaddr_in serv_addr;

    // congif server address
    int len_inet = sizeof(struct sockaddr_in);
    memset(&serv_addr, 0, len_inet);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    int IPAddrStatus = inet_aton(ip_addr, &serv_addr.sin_addr);
    if (IPAddrStatus == 0)
    {
        // perror("inet_aton error");
        LOG_ERROR("errno number is %d\n", errno);
        return (-1);
    }

    // connect server
    if (connect(sockfd, (struct sockaddr *)&serv_addr, len_inet) == -1)
    {
        // perror("connect error");
        LOG_ERROR("errno number is %d\n", errno);
        return (-1);
    }
    return 0;
}
/* ----------------------------------------------------------------------------------- */

/**
 * @brief 接收数据
 *
 * @param sockfd
 * @param recv_buf
 * @param RECV_MAXDATASIZE
 *
 * @return int
 */
// rcv data
int recv_data(int sockfd, char *recv_buf, int RECV_MAXDATASIZE)
{
    int recvbytes;
    recvbytes = recv(sockfd, recv_buf, RECV_MAXDATASIZE, 0);

    if (recvbytes == -1)
    {
        // perror("recv error");
        LOG_ERROR("errno number is %d\n", errno);
        return (-1);
    }
    return recvbytes;
}
/* ----------------------------------------------------------------------------------- */

/**
 * @brief 发送数据
 *
 * @param sockfd
 * @param send_buf
 * @param RECV_MAXDATASIZE
 *
 * @return int
 */
// send data
int send_data(int sockfd, char *send_buf, int RECV_MAXDATASIZE)
{
    int sendbytes;
    sendbytes = send(sockfd, send_buf, RECV_MAXDATASIZE, 0);
    if (sendbytes == -1)
    {
        // perror("recv error");
        LOG_ERROR("errno number is %d\n", errno);
        return (-1);
    }
    return sendbytes;
}
/* ----------------------------------------------------------------------------------- */

/**
 * @brief 测试网络是否通畅
 *
 * @param ip
 *
 * @return int
 */
// ping network
int link_network(char *ip)
{
    int nCount = 0;
    pid_t pid;
    while (nCount < 5)
    {
        if (execl("/bin/ping", "ping", "-c1", ip, (char *)0) < 0)
        {
            LOG_ERROR("execl error\n");
            LOG_ERROR("errno=%d\n", errno);
            exit(1);
        }
        else
            return 0;
        sleep(2);
        nCount++;
    }
    return (-1);
}
/* ----------------------------------------------------------------------------------- */

/**
 * @brief 绑定特定的端口
 *
 * @param sockfd
 * @param BIND_PORT
 * @param local_addr
 *
 * @return int
 */
// bind specified
int bind_specified(int sockfd, int BIND_PORT, struct sockaddr_in local_addr)
{
    local_addr.sin_family = PF_INET;
    local_addr.sin_port = htons(BIND_PORT);
    local_addr.sin_addr.s_addr = INADDR_ANY;
    bzero(&(local_addr.sin_zero), 8);

    // bind
    if (bind(sockfd, (struct sockaddr *)&local_addr, sizeof(struct sockaddr)) == -1)
    {
        // perror("bind error");
        LOG_ERROR("errno number is %d\n", errno);
        return (-1);
    }
    return 0;
}
/* ----------------------------------------------------------------------------------- */

/**
 * @brief 监听端口
 *
 * @param sockfd
 * @param BACKLOG
 *
 * @return int
 */

// listen specified
int listen_specified(int sockfd, int BACKLOG)
{
    // detect bind port
    if (listen(sockfd, BACKLOG) == -1)
    {
        // perror("listen error");
        LOG_ERROR("errno number is %d\n", errno);
    }
    return 0;
}
/* ----------------------------------------------------------------------------------- */

/**
 * @brief 接受请求
 *
 * @param sockfd
 * @param clientfd
 * @param remote_addr
 *
 * @return int
 */
// accept request
int accept_request(int sockfd, int clientfd, struct sockaddr_in remote_addr)
{
    unsigned int addr = sizeof(remote_addr);
    if ((clientfd = accept(sockfd, (struct sockaddr *)&remote_addr, &addr)) == -1)
    {
        // perror("listen error!");
        LOG_ERROR("errno number is %d\n", errno);
    }
    return clientfd;
}
/* ----------------------------------------------------------------------------------- */
