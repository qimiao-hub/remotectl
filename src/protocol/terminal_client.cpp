#include "./terminal_client.h"


terminal_client::terminal_client()
{


}

sa_comm_socket_client *terminal_client::sa_comm_socket_client_new(const char * server_name,
                                                                  const char * local_name,
                                                                  bool b_fix_port)
{
    sa_comm_socket_client *socket_client = (sa_comm_socket_client *)calloc(1, sizeof(sa_comm_socket_client));
    if (!socket_client) {
        printf("%s: calloc failed.\n", __func__);
        goto __error;
    }

    socket_client->b_connected = 0;

    socket_client->socket = m_Sa_Comm.sa_comm_socket_new(server_name,
                                                         local_name,
                                                         b_fix_port);
    if (!socket_client->socket) {
        printf("%s: sa_comm_socket_new failed.\n", __func__);
        goto __error;
    }

    return socket_client;

__error:
    sa_comm_socket_client_release(socket_client);
    return NULL;

}

void terminal_client::sa_comm_socket_client_release(sa_comm_socket_client *socket_client)
{
    if (socket_client) {

        m_Sa_Comm.sa_comm_socket_release(socket_client->socket);
        free(socket_client);
    }

}

int terminal_client::comm_socket_client_do_connect(sa_comm_socket_client *socket_client)
{
    if (!socket_client || !socket_client->socket) {
        return 0;
    }

    /* try to open socket client */
    if (socket_client->socket_fd <= 0) {
        if ((socket_client->socket_fd = m_Sa_Comm.sa_comm_socket_client_open(socket_client->socket)) <= 0) {
            printf("%s: open socket failed.\n", __func__);
            return 0;
        }
        printf("%s: open socket successful, fd: %d.\n", __func__, socket_client->socket_fd);

        if(socket_client->socket->b_fix_port_ip)
        {
            struct sockaddr_in clientaddr;
            bzero(&clientaddr,sizeof(clientaddr));
            clientaddr.sin_family = AF_INET;
            clientaddr.sin_addr.s_addr=inet_addr(socket_client->socket->fix_ip);
            clientaddr.sin_port=htons(socket_client->socket->fix_port);


            bool bReuseaddr=true;
            setsockopt (socket_client->socket_fd,SOL_SOCKET ,SO_REUSEADDR,
                        (const char*)&bReuseaddr,sizeof(bool));

            if (bind(socket_client->socket_fd, (struct sockaddr *) &clientaddr,
                     sizeof(clientaddr)) < 0)
            {
                 perror("bind");
            }
        }
    }

    if(0 == socket_client->b_connected)
    {
        /* try to connect tcp/ip */
        printf("try connect \n");

         int ret_tmp = m_Sa_Comm.sa_comm_socket_client_connect(socket_client->socket, socket_client->socket_fd);
        if (ret_tmp==-1) {
            printf("%s: connect failed.\n", __func__);
            return 0;
        }
        socket_client->b_connected = 1;
    }
    else{
        return 0;
    }

    printf("%s: connect server successful.\n", __func__);
    return 1;
}

int terminal_client::sa_comm_socket_client_send(sa_comm_socket_client *socket_client, char *data, int data_len)
{
    if (!socket_client || !data || data_len <= 0) {
            return 0;
        }

    if (!m_Sa_Comm.sa_comm_socket_write(socket_client->socket_fd, data, data_len))
    {
        comm_socket_client_do_disconnect(socket_client);
        return 0;
    }
    return 1;


}

int terminal_client::sa_comm_socket_client_read(int fd, void *userdata)
{
    sa_comm_socket_client *socket_client = (sa_comm_socket_client *)userdata;
    if (!socket_client) {
        printf("socket_client error\n");
        return 0;
    }

    int iRet = 0;
    uint32_t length = 0;

    /* read message header */
    iRet = m_Sa_Comm.sa_comm_socket_read(fd, socket_client->recv_buffer, sizeof(socket_client->recv_buffer),
                               sizeof(WP_Header_st));
    printf("recv = %s",socket_client->recv_buffer);
    if (0 >= iRet)
    {
        printf("%s: read length failed.\n", __func__);

        comm_socket_client_do_disconnect(socket_client);
        return 0;
    }
    printf("recv len = %d 11",length);

    
    return length;

}

void terminal_client::comm_socket_client_do_disconnect(sa_comm_socket_client *socket_client)
{
    if (!socket_client) {
        return;
    }

    if (socket_client->socket_fd > 0) {
        m_Sa_Comm.sa_comm_socket_close(socket_client->socket_fd);
        socket_client->socket_fd = 0;
    }

    socket_client->b_connected = 0;

}


