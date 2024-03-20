#include "sa_socket_comm.h"



sa_socket_comm::sa_socket_comm()
{


}
/**
* @funcName   sa_comm_socket_new
* @Brief:
* @Param:
* @Param:
* @Return:
* @date       2020-11-29
*/
sa_comm_socket *sa_socket_comm::sa_comm_socket_new(const char *server_name,
                                                   const char *client_name,
                                                   bool b_fix_client)
{
    sa_comm_socket *sock = (sa_comm_socket *)calloc(1, sizeof(sa_comm_socket));
    if (!server_name) {
        printf("%s: invalid parameter.\n", __func__);
     //   return NULL;
        goto __invalid;
    }


    if (!sock) {
        printf("%s: calloc failed.\n", __func__);
        goto __error;
    }

    sock->server_name = strdup(server_name);
    if (!sock->server_name) {
        printf("%s: save server name failed.\n", __func__);
        goto __error;
    }

    sock->b_tcpip = socket_if_server_name_tcpip(server_name, sock->ip, &sock->port);
    if (sock->b_tcpip) {
        printf("%s: ip: %s, port: %d.\n", __func__, sock->ip, sock->port);
    }

    sock->b_fix_port_ip = b_fix_client;
    if(sock->b_fix_port_ip)
    {
        socket_if_server_name_tcpip(client_name, sock->fix_ip, &sock->fix_port);
    }

    return sock;

__error:
    sa_comm_socket_release(sock);
__invalid:
    return NULL;
}
/**
* @funcName   sa_comm_socket_release
* @Brief:     释放sa_comm_socket结构体空间和结构体指针空间
* @Param:
* @Param:
* @Return:
* @date       2020-11-29
*/
void sa_socket_comm::sa_comm_socket_release(sa_comm_socket *sock)
{
    if (sock) {
            if (sock->server_name) {
                free(sock->server_name);
            }
            free(sock);
        }

}
/**
* @funcName   sa_comm_socket_server_open
* @Brief:
* @Param:
* @Param:
* @Return:
* @date       2020-11-29
*/
int sa_socket_comm::sa_comm_socket_server_open(sa_comm_socket *sock, int max_connection)
{
    if (!sock) {
            printf("%s: invalid parameter.\n", __func__);
            return 0;
        }

    if (sock->b_tcpip) {
        return socket_server_open_tcpip(sock->ip, sock->port, max_connection);
    };

}

int sa_socket_comm::sa_comm_socket_server_accept(sa_comm_socket *sock, int sock_fd)
{
    if (!sock) {
        printf("%s: invalid parameter.\n", __func__);
        return 0;
    }

    if (sock->b_tcpip) {
        return socket_server_accept_tcpip(sock_fd);
    };

}

int sa_socket_comm::sa_comm_socket_client_open(sa_comm_socket *sock)
{
    if (!sock) {
        printf("%s: invalid parmeter.\n", __func__);
        return 0;
    }

    if (sock->b_tcpip) {
        return socket_client_open_tcpip();
    }


}

int sa_socket_comm::sa_comm_socket_client_connect(sa_comm_socket *sock, int fd)
{
    if (!sock) {
            printf("%s: invalid parmeter.\n", __func__);
            return 0;
        }

        if (sock->b_tcpip) {
            printf("connect tcp \n");
            return socket_client_connect_tcpip(fd, sock->ip, sock->port);
        };

}

int sa_socket_comm::sa_comm_socket_close(int fd)
{
    if (fd > 0) {
            printf("%s: fd: %d.\n", __func__, fd);
            close(fd);
        };

}

int sa_socket_comm::sa_comm_socket_read(int fd, char *buffer, unsigned long buffer_size, unsigned long len)
{
    if (fd <= 0 || !buffer || len <= 0) {
        printf("%s: invalid parameter.\n", __func__);
        return -1;
    }

    if (buffer_size < len) {
        printf("%s: buffer size too small.\n", __func__);
        return -1;
    }

    unsigned long read_size = 0;
    int cur_size = 0;
    int readcnt = 0;

    while (read_size < len) {
        cur_size = read(fd, buffer + read_size, len - read_size);

        //printf("%s: cur_size = %d \n", __func__, cur_size);
        if (cur_size == 0) {
            return 0;
        } else if (cur_size < 0 ) {
            usleep(1000);
            readcnt++;
            if (3000 < readcnt)
            {
                /* 防止读出错时造成阻塞 */
                break;
            }
            continue;
        }
        read_size += cur_size;
    }

    #ifdef PROTOCOL_DEBUG
    int index = 0;
    printf("Read socket: data_len: %d,[", len, buffer);
    for(index = 0; index < len; index++)
    {
        printf("%02X ",buffer[index]);
    }
    printf("]\r\n");

    printf("read_size = %d \r\n", read_size);
    #endif

    return read_size;


}

int sa_socket_comm::sa_comm_socket_write(int fd, char *buffer, unsigned long len)
{
    if (fd <= 0 || !buffer || len <= 0) {
        printf("%s: invalid parameter.\n", __func__);
        return 0;
    }

    unsigned long write_size = 0;
    int cur_size = 0;

    #ifdef PROTOCOL_DEBUG
    int index = 0;
    printf("Write socket: data_len: %d,[", len, buffer);
    for(index = 0; index < len; index++)
    {
        printf("%02X ", buffer[index]);
    }
    printf("]\r\n");
    #endif

    while (write_size < len)
    {
        cur_size = write(fd, buffer + write_size, len - write_size);
        if (cur_size <= 0)
        {
            return 0;
        }

        write_size += cur_size;
    }

    return write_size;

}
/**
* @funcName   socket_if_server_name_tcpip
* @Brief:     传入地址字符串和ip端口地址指针生成ip和port
* @Param:
* @Param:
* @Return:    int
* @date       2020-11-29
*/
int sa_socket_comm::socket_if_server_name_tcpip(const char *server_name, char *ip, int *port)
{
    char *pos = NULL, p_buf[10];
    int len;

    pos = (char *)index(server_name, ':');
    if (!pos) {
        return 0;
    }

    if (strlen(server_name) > 21) {
        return 0;
    }

    len = pos - server_name;
    if (len > 15) {
        return 0;
    }

    memcpy(ip, server_name, len);
    ip[len] = '\0';
    if(INADDR_NONE == inet_addr(ip)) {
        return 0;
    }

    memcpy(p_buf, pos+1, strlen(server_name)-len-1);
    *port = atoi(p_buf);
    if(*port > 65535) {
        return 0;
    }

    return 1;

}
/**
* @funcName   socket_server_open_tcpip
* @Brief:     build server socket and listen
* @Param:
* @Param:
* @Return:
* @date       2020-11-29
*/
int sa_socket_comm::socket_server_open_tcpip(const char *ip, int port, int max_connection)
{
    int fd = -1;
    struct sockaddr_in addr;

//    fd = socket(AF_INET, SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC, 0);
    fd = socket(AF_INET, SOCK_STREAM|SOCK_CLOEXEC, 0);
    if (fd < 0) {
        printf("%s: socket failed.\n", __func__);
        return -1;
    }

    memset(&addr, 0x00, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip);
    addr.sin_port = htons(port);

    int on = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
        printf("%s: setsockopt SO_REUSEADDR failed.\n", __func__);
        goto __error;
    }

    if (bind(fd, (struct sockaddr *)&addr, sizeof(struct sockaddr)) < 0) {
        printf("%s: bind failed, errno: %d, %s.\n", __func__, errno, strerror(errno));
        goto __error;
    }

    if (listen(fd, max_connection) < 0) {
        printf("%s: listen failed.\n", __func__);
        goto __error;
    }

    return fd;

__error:
    sa_comm_socket_close(fd);
    return -1;

}
/**
* @funcName   socket_server_accept_tcpip
* @Brief:     no block server accept client req
* @Param:
* @Param:
* @Return:
* @date       2020-11-29
*/
int sa_socket_comm::socket_server_accept_tcpip(int sock_fd)
{
    struct sockaddr_in in;
    socklen_t len = sizeof(struct sockaddr_in);

    int fd = accept4(sock_fd, (struct sockaddr *)&in, &len, SOCK_NONBLOCK|SOCK_CLOEXEC);

    return fd;

}
/**
* @funcName   socket_client_open_tcpip
* @Brief:     build server socket
* @Param:
* @Param:
* @Return:
* @date       2020-11-29
*/
int sa_socket_comm::socket_client_open_tcpip()
{
    int fd = socket(PF_INET, SOCK_STREAM, 0);
    printf("fd = %d\n", fd);
    return fd;

}
/**
* @funcName   socket_client_connect_tcpip
* @Brief:     client connect server
* @Param:
* @Param:
* @Return:
* @date       2020-11-29
*/
int sa_socket_comm::socket_client_connect_tcpip(int sock_fd, const char *ip, int port)
{
    struct sockaddr_in addr;
    memset(&addr, 0x00, sizeof(struct sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip);
    addr.sin_port = htons(port);
    printf("ip = %s,port = %d\n",ip,port);

    //设置为非阻塞模式
    fcntl(sock_fd,F_SETFL,fcntl(sock_fd,F_GETFL,0)|O_NONBLOCK);
    
    int connected = connect(sock_fd, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
    int ret = -1;
    if (0 != connected )
    {
        if(errno != EINPROGRESS)
            printf("connect error :%s\n",strerror(errno));
        else
        {
            struct timeval tm = {10, 0};
            fd_set wset,rset;
            FD_ZERO(&wset);
            FD_ZERO(&rset);
            FD_SET(sock_fd,&wset);
            FD_SET(sock_fd,&rset);
            long t1 = time(NULL);
            int res = select(sock_fd + 1, &rset, &wset, NULL, &tm);
            long t2 = time(NULL);
            printf("interval time: %ld\n", t2 - t1);
            if(res < 0)
            {
                printf("network error in connect\n");
            }
            else if(res == 0)
            {
                printf("connect time out\n");
            }
            else if (1 >= res)
            {
                if(FD_ISSET(sock_fd,&wset))
                {
                    printf("connect succeed.\n");
                    fcntl(sock_fd,F_SETFL,fcntl(sock_fd,F_GETFL,0) & ~O_NONBLOCK);
                    ret = 0;
                }
                else
                {
                    printf("other error when select:%s\n",strerror(errno));
                }
            }
        }
    }
    return ret;
}
