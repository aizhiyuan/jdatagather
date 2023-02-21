#ifndef H_COLLECTDATA_NETWORK_H
#define H_COLLECTDATA_NETWORK_H

#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

int opensock();
int req_conn(int sockfd, const char *ip_addr, int port);
int recv_data(int sockfd, char *recv_buf, int RECV_MAXDATASIZE);
int send_data(int sockfd, char *send_buf, int RECV_MAXDATASIZE);
int link_network(char *ip);
int bind_specified(int sockfd, int BIND_PORT, struct sockaddr_in local_addr);
int listen_specified(int sockfd, int BACKLOG);
int accept_request(int sockfd, int clientfd, struct sockaddr_in remote_addr);

#endif
