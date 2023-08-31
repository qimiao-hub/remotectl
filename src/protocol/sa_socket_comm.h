#ifndef SA_SOCKET_COMM_H
#define SA_SOCKET_COMM_H

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include <time.h>

#define SCOMM_IP_BUF_SIZE   128
typedef struct _sa_comm_socket sa_comm_socket;

struct _sa_comm_socket {
    char *server_name;
    int b_tcpip;
    char ip[SCOMM_IP_BUF_SIZE];
    int port;

    bool b_fix_port_ip;
    char fix_ip[SCOMM_IP_BUF_SIZE];
    int fix_port;
};



class sa_socket_comm
{
public:
    sa_socket_comm();



    sa_comm_socket *sa_comm_socket_new(const char *server_name,const char *client_name,bool b_fix_client);
    void sa_comm_socket_release(sa_comm_socket *sock);
    int sa_comm_socket_server_open(sa_comm_socket *sock, int max_connection);
    int sa_comm_socket_server_accept(sa_comm_socket *sock, int sock_fd);
    int sa_comm_socket_client_open(sa_comm_socket *sock);
    int sa_comm_socket_client_connect(sa_comm_socket *sock, int fd);
    int sa_comm_socket_close(int fd);
    int sa_comm_socket_read(int fd, char *buffer, unsigned long buffer_size, unsigned long len);
    int sa_comm_socket_write(int fd, char *buffer, unsigned long len);

    int socket_if_server_name_tcpip(const char *server_name, char *ip, int *port);

    int socket_server_open_tcpip(const char *ip, int port, int max_connection);
    int socket_server_accept_tcpip(int sock_fd);
    int socket_client_open_tcpip(void);
    int socket_client_connect_tcpip(int sock_fd, const char *ip, int port);


};

#endif // SA_SOCKET_COMM_H
