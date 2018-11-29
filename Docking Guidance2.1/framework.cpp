//
// Created by LKK on 18-6-23.
//
#include "framework.h"
#include <math.h>

#include<sys/socket.h>
#include<sys/types.h>
#include<sys/un.h>
#include <unistd.h>
#include <linux/tcp.h>
#include <netinet/in.h>

#include "GlobleData.h"

#define UNIX_DOMAIN_SEND "/tmp/LidarToManage.domain"
#define UNIX_DOMAIN_RECV "/tmp/ManageToLidar.domain"



using namespace rapidjson;

#define DISPLAYPORT 9999
//#define DISPLAYIP "192.168.1.118"
#define DISPLAYIP "127.0.0.1"
#define BUFSIZE 512
#define FALSE 0

std::mutex _mtx;
int _READYTOSENDFLAG = 0;
DISPLAYINFO _displayinfo;

extern struct control_msg con_msg;


//本地socket发送数据
void Send_localsocket(){


    std::cout << "已经进入发送数据的函数当中" << std::endl;


    int connect_fd;
    int ret;
    char send_buff[1024];
    static struct sockaddr_un srv_addr;

    //--------------
//    int len_craft;
//    char buf_tmp[20];
//    Value flight, position;

    Document doc;
    doc.SetObject();
    Document::AllocatorType &allocator = doc.GetAllocator(); //获取分配器
    /*initialzation*/
    _displayinfo.craft = "A380";
    _displayinfo.distance = 50.0;
    _displayinfo.speed = 5.31;
    _displayinfo.position = "left";
    _displayinfo.offset = 3.0;

    Value flight, position;
    char buf_tmp[20];
    memset(buf_tmp, 0, sizeof(buf_tmp));
    int len_craft = sprintf(buf_tmp, "%s", _displayinfo.craft.c_str());
    flight.SetString(buf_tmp, len_craft, allocator);
    memset(buf_tmp, 0, sizeof(buf_tmp));
    int len_position = sprintf(buf_tmp, "%s", _displayinfo.position.c_str());
    position.SetString(buf_tmp, len_position, allocator);
    memset(buf_tmp, 0, sizeof(buf_tmp));


    doc.AddMember("@table",1,allocator);
    doc.AddMember("@src","lidar",allocator);
    doc.AddMember("workstatus",1,allocator);
    doc.AddMember("detectflag", _displayinfo.detectflag, allocator);
    doc.AddMember("craft", flight, allocator);
    doc.AddMember("position", position, allocator);
    doc.AddMember("distance", _displayinfo.distance, allocator);
    doc.AddMember("speed", _displayinfo.speed, allocator);
    doc.AddMember("offset", _displayinfo.offset, allocator);


    //定义发送消息队列变量
    int msgid_2;
    struct my_msg_st some_data_2;
    long int msg_to_receive_2 = 0;
    //创建消息队列
    msgid_2 = msgget((key_t) 1235, 0666 | IPC_CREAT);
    if (msgid_2 == -1) {
        fprintf(stderr, "发送线程 接收消息队列msgget failed with error: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    while (1) {

        //接收到主线程的返回信息，并将相关信息发送给前端；
        if (msgrcv(msgid_2, (void *) &some_data_2, BUFSIZ, msg_to_receive_2, IPC_NOWAIT) == -1)
//        if (msgrcv(msgid_2, (void *) &some_data_2, BUFSIZ, msg_to_receive_2, 0) == -1)
        {
//            fprintf(stderr, "msgrcv failed with error: %d\n", errno);
//            exit(EXIT_FAILURE);
        } else
        {
            printf("发送线程接收到的消息队列的数据为： %s\n", some_data_2.some_text);
            connect_fd=socket(PF_UNIX,SOCK_STREAM,0);
            if(connect_fd<0){
                perror("cannot creat socket");
                break;
            }
            srv_addr.sun_family=AF_UNIX;
            strcpy(srv_addr.sun_path,UNIX_DOMAIN_SEND);


            ret=connect(connect_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
            if (ret<0){
//          perror("cannot connect server");
                printf("cannot connect to 控制软件\n");
                close(connect_fd);
//            break;
                continue;
            }

            write(connect_fd,some_data_2.some_text, sizeof(some_data_2.some_text));
            close(connect_fd);
            continue;


        }

//        _READYTOSENDFLAG = 0;
        //发送算法的结果数据
        if (_READYTOSENDFLAG)
        {
//          std::cout<<"ready"<<std::endl;
//            _mtx.lock();
            _READYTOSENDFLAG = 0;
//            _mtx.unlock();



            connect_fd=socket(PF_UNIX,SOCK_STREAM,0);
            if(connect_fd<0){
                perror("cannot creat socket");
                break;
            }
            srv_addr.sun_family=AF_UNIX;
            strcpy(srv_addr.sun_path,UNIX_DOMAIN_SEND);


            ret=connect(connect_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
            if (ret<0){
                perror("cannot connect server");
                close(connect_fd);
                continue;
            }

            //---------------------------------------------
            len_craft = sprintf(buf_tmp, "%s", _displayinfo.craft.c_str());
            flight.SetString(buf_tmp, len_craft, allocator);
            doc["craft"] = flight;
            int len_position = sprintf(buf_tmp, "%s", _displayinfo.position.c_str());
            position.SetString(buf_tmp, len_position, allocator);
            memset(buf_tmp, 0, sizeof(buf_tmp));
            doc["position"] = position;
            _displayinfo.distance = round(_displayinfo.distance * 100) / 100;
            doc["distance"] = _displayinfo.distance;
            doc["speed"] = _displayinfo.speed;
            doc["offset"] = _displayinfo.offset;
            doc["detectflag"] = _displayinfo.detectflag;
            StringBuffer buffer;
            Writer<StringBuffer> writer(buffer);
            doc.Accept(writer);

            std::cout<<"算法检测要发送的数据为："<<buffer.GetString()<<std::endl;



            //---------------------------------------------
//            char *ch = "{\"craft\":\"WalkTest\",\"distance\":27.0,\"speed\":0.0,\"position\":\"LEFT\",\"offset\":1.3835633748444607,\"detectflag\":1}";
//            memset(send_buff,0,1024);
//            strcpy(send_buff,ch);
            //send info server
            write(connect_fd,buffer.GetString(),strlen(buffer.GetString()));
            close(connect_fd);
        }

        usleep(1000*100);
    }


}


//本地socket接收数据
void Recv_localSocket()
{
    socklen_t clt_addr_len;
    int listen_fd;
    int com_fd;
    int ret;
    int i;
    static char rcv_buff[1024];
    int len;
    struct sockaddr_un clt_addr;
    struct sockaddr_un srv_addr;
    srv_addr.sun_family=AF_UNIX;
    strncpy(srv_addr.sun_path,UNIX_DOMAIN_RECV,sizeof(srv_addr.sun_path)-1);
    unlink(UNIX_DOMAIN_RECV);

    /////////////////////消息队列的初始化///////////
    struct my_msg_st some_data;
    int msgid;
    char buffer[1024];
    //创建消息队列
    msgid = msgget((key_t) 1234, 0666 | IPC_CREAT);
    if (msgid == -1) {
        fprintf(stderr, "msgget failed with error:%d\n", errno);
        exit(EXIT_FAILURE);
    }



    while (1) {

        listen_fd=socket(AF_UNIX,SOCK_STREAM,0);
        if(listen_fd<0){
            perror("connect creat communication socket");
        }

        //bind sockfd&addr
        ret=bind(listen_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
        if(ret<0){
            perror("cannot bind server socket");
            close(listen_fd);
            unlink(UNIX_DOMAIN_RECV);
            break;
        }

        //listen sockfd
        ret=listen(listen_fd,1);
        if(ret<0){
            perror("cannot listen sockfd");
            close(listen_fd);
            unlink(UNIX_DOMAIN_RECV);
            break;
        }

        //have connect requst use accept
        len=sizeof(clt_addr);
        com_fd=accept(listen_fd,(struct sockaddr*)&clt_addr,(socklen_t *)&len);
        if(com_fd<0){
            perror("cannot accept requst " ) ;
            close(listen_fd);
            unlink(UNIX_DOMAIN_RECV);
            break;
        }

        //read and printf client send info
        for(i=0;i<4;i++){
            memset(rcv_buff,0,1024);
            int num = read(com_fd,rcv_buff,sizeof(rcv_buff));
            if(0 < num)
            {
                printf("message from client %d : %s\n",num,rcv_buff);

                //写入到消息队列
                some_data.my_msg_type = 1;
                strcpy(some_data.some_text, rcv_buff);
                if (msgsnd(msgid, (void *) &some_data, 1024, 0) == -1) {
                    fprintf(stderr, "msgsed failed\n");
                    exit(EXIT_FAILURE);
                }
            }

        }
        close(com_fd);
        close(listen_fd);
        unlink(UNIX_DOMAIN_RECV);
        usleep(1000*1000);

    }

    return ;
}
