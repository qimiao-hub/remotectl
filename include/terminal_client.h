#ifndef TERMINAL_CLIENT_H
#define TERMINAL_CLIENT_H

#include "./sa_socket_comm.h"
#include "./protoDef.h"
//#define SA_COMM_SOCKET_SERVER_NAME "127.0.0.1:11222"
//#define SA_COMM_SOCKET_SERVER_NAME      "59.173.243.67:8999"
#define SA_COMM_SOCKET_SERVER_NAME      "114.255.128.132:11245"
#define BUF_SIZE        65535
#define SA_COMM_SOCKET_CONNECT_INTERVAL     1000

typedef struct _sa_comm_socket_client sa_comm_socket_client;
typedef void (*sa_comm_socket_client_cb)(char *data, int data_len, void *userdata);
struct _sa_comm_socket_client {
    sa_comm_socket *socket;
    int socket_fd;
    int b_connected;
    char recv_buffer[BUF_SIZE];
};

class terminal_client
{
public:
    terminal_client();
    sa_comm_socket_client *sa_comm_socket_client_new(const char * server_name,
                                                     const char * local_name,
                                                     bool b_fix_port);
    void sa_comm_socket_client_release(sa_comm_socket_client *socket_client);
    int comm_socket_client_do_connect(sa_comm_socket_client *socket_client);
    int sa_comm_socket_client_read(int fd, void *userdata);
    int sa_comm_socket_client_send(sa_comm_socket_client *socket_client, char *data, int data_len);
    void comm_socket_client_do_disconnect(sa_comm_socket_client *socket_client);
    
    sa_socket_comm m_Sa_Comm;

};

#endif // TERMINAL_CLIENT_H
