/*
* Company: LES
* Auth: LKK GX LL
* Date: 18/06/23
* Decription:
*/

#include <iostream>

//#include "AircraftDetect.h"
#include "WalkTest.h"
#include "framework.h"

#include <iostream>
#include <cstring>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <sqlite3.h>
#include <X11/Intrinsic.h>

#include <sstream>
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "GlobleData.h"
using namespace std;

using namespace rapidjson;

std::string hdlCalibration("");
std::string pcapFile("walk2.pcap");//airplane_06130151 airplane_06112142 walktest_5.8_0613_1 walktest_5.8_0613_2
//std::string typeFile("class_name.txt");
//std::string aircraftData("aircraft_data.txt");




const char *type_file = "class_name.txt";
const char *aircraft_data = "aircraft_data.txt";
std::string ip("");
//std::string ip("192.168.20.100");
int port = 8080;
pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler("intensity");

bool _aircraft_detect = false; //false
bool _run_flag = false;

sqlite3 *db = NULL;     //数据库链接 声明为全局

extern struct control_msg con_msg;  //控制信息结构体变量
extern int workMode_ = 0;


void LidarHandler() {

    if (1 == workMode_) {
//        if (ip == "") {
//            std::cout << pcapFile << std::endl;
//            AircraftDetect aircraft_detect(hdlCalibration, pcapFile, type_file, aircraft_data, color_handler);
//            aircraft_detect.run();
//        } else {
//            std::cout << "ip: " << ip << "port: " << port << std::endl;
//            AircraftDetect aircraft_detect(hdlCalibration, type_file, aircraft_data, ip, port, color_handler);
//            aircraft_detect.run();
//        }
    } else if (2 == workMode_){
        if (ip == "") {
            std::cout << pcapFile << std::endl;
            WalkTest walktest(hdlCalibration, pcapFile, color_handler);
            walktest.run();
            cout << "************************************" << endl;
        } else {
            std::cout << "ip: " << ip << "port: " << port << std::endl;
            WalkTest walktest(hdlCalibration, ip, port, color_handler);
            walktest.run();

        }
    }
}


//点云处理线程
void PCLDealingThread() {

    while (1) {
        if (_run_flag) {
            LidarHandler();
            _run_flag = false;
        } else {
            usleep(100 * 1000);
        }
    }
}


/*
 * 数据库初始化（创建数据库 、AIRPLANEMODEL_TAB数据表）
 */
void initSqlite3Bases() {
    char *zErrMsg = 0;
    int rc;
    //打开指定的数据库文件,如果不存在将创建一个同名的数据库文件
    rc = sqlite3_open("/usr/local/LES/AIRPORT.db", &db);
    if (rc) {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
        return;
    }
    const char *sql;

    //开启外键限制
    sql = "PRAGMA foreign_keys = ON;";
    sqlite3_exec(db, sql, NULL, NULL, &zErrMsg);
//    std::cout<<"开启外键限制"<<std::endl;


    //创建参数存储PARAMETER_TAB表
    sql = "CREATE TABLE PARAMETER_TAB(ID int PRIMARY KEY,CLUSTERMIN int,CLUSTERMAX int,TOLERANCE float,CLIPMIN float,CLIPMAX float,CLIPLEFT float,CLIPRIGHT float);";
    sqlite3_exec(db, sql, NULL, NULL, &zErrMsg);
    sql = "INSERT INTO \"PARAMETER_TAB\" VALUES(1, 1, 2, 0.1, 0.2, 0.3, 0.4, 0.5 );";
    sqlite3_exec(db, sql, 0, 0, &zErrMsg);

    //创建AIRPLANEMODEL_TAB表
    sql = "CREATE TABLE AIRPLANEMODEL_TAB(AIRPLANE_MODEL VARCHAR(30) PRIMARY KEY,WING_LENGTH float,ENGINE_INNER float,ENGINE_OUTER float,AIRPLANE_LENGTH float,NOSE_HEIGHT float);";
    sqlite3_exec(db, sql, NULL, NULL, &zErrMsg);
    //插入数据
    sql = "INSERT INTO \"AIRPLANEMODEL_TAB\" VALUES('A380-300' , 1.5 , 22.01, 18.9, 50.1, 22.2 );";
    sqlite3_exec(db, sql, 0, 0, &zErrMsg);
}


/*
 *创建泊位信息表，参数为 泊位名称
 * 参数：泊位名称 berthName(不用)
 */
void CreateBerthInfo_table() {
    char *zErrMsg;
    const char *sql;
    //创建表
    sql = "CREATE TABLE BERTH_TAB(AIRPLANE_MODEL VARCHAR(30) PRIMARY KEY,STOP1_X float,STOP1_Y float,STOP2_X float,STOP2_Y float,CENTER1_X float,CENTER1_Y float,CENTER2_X float,CENTER2_Y float,FOREIGN KEY(AIRPLANE_MODEL) REFERENCES AIRPLANEMODEL_TAB(AIRPLANE_MODEL));";
    sqlite3_exec(db, sql, NULL, NULL, &zErrMsg);
    //插入数据
    sql = "INSERT INTO \"BERTH_TAB\" VALUES('A380-300',1.5 ,22.01, 18.9, 50.1, 22.2, 22.01, 18.9, 50.1 );";
    sqlite3_exec(db, sql, 0, 0, &zErrMsg);
//    std::cout << "泊位信息表中插入数据：" << zErrMsg << std::endl;

}



/*
 * 查询参数的信息 测试用
 */
void selectInfoFromPARAMETER_TAB()
{
    char *zErrMsg;
    const char *sql;
    int nrow = 0, ncolumn = 0;
    char **azResult; //二维数组存放结果
    sql = "SELECT * FROM PARAMETER_TAB  ";
    sqlite3_get_table(db, sql, &azResult, &nrow, &ncolumn, &zErrMsg);
    int i = 0;
    printf("row:%d column=%d \n", nrow, ncolumn);
    printf("\nThe result of querying is : \n");
    for (i = 0; i < (nrow + 1) * ncolumn; i++)
        printf("azResult[%d] = %s\n", i, azResult[i]);
    printf("zErrMsg = %s \n", zErrMsg);
}


/*
 * 数据库中查询机型信息 测试用
 */
void selectInfoFromAIRPLANEMODEL_TAB()
{
    char *zErrMsg;
    const char *sql;
    int nrow = 0, ncolumn = 0;
    char **azResult; //二维数组存放结果
    sql = "SELECT * FROM AIRPLANEMODEL_TAB  ";
    sqlite3_get_table(db, sql, &azResult, &nrow, &ncolumn, &zErrMsg);
    int i = 0;
    printf("row:%d column=%d \n", nrow, ncolumn);
    printf("\nThe result of querying is : \n");
    for (i = 0; i < (nrow + 1) * ncolumn; i++)
        printf("azResult[%d] = %s\n", i, azResult[i]);
    printf("zErrMsg = %s \n", zErrMsg);
}

/*
 *  查询泊位信息表中的内容 测试用
 */
void selectInfoFromBerthTAB(char *airplane_model) {
    char *zErrMsg;
    const char *sql;
    //创建表
    sql = "SELECT * FROM BERTH_TAB WHERE AIRPLANE_MODEL = ?";
    sqlite3_exec(db, sql, NULL, NULL, &zErrMsg);
//    int i = 0;
//    printf("row:%d column=%d \n", nrow, ncolumn);
//    printf("\nThe result of querying is : \n");
//    for (i = 0; i < (nrow + 1) * ncolumn; i++)
//        printf("azResult[%d] = %s\n", i, azResult[i]);
//    printf("zErrMsg = %s \n", zErrMsg);

}


int main() {

    initSqlite3Bases();
    CreateBerthInfo_table();                         //泊位号

//    selectInfoFromPARAMETER_TAB();

//    std::thread recvHandler(Recvsocket_UDP);             //UDP方式接受数据
//    std::thread InformationDisplayHandler(socket_UDP);   //UDP方式发送数据
    std::thread PclDealthreadHandler(PCLDealingThread);    //目标识别线程
    std::thread sendLocalSOcket_handle(Send_localsocket);  //数据发送线程
    std::thread recvLocalSocket_handle(Recv_localSocket);  //数据接收线程


    //定义接收消息队列变量
    int msgid;
    struct my_msg_st some_data;
    long int msg_to_receive = 0;
    //创建消息队列
    msgid = msgget((key_t) 1234, 0666 | IPC_CREAT);
    if (msgid == -1) {
        fprintf(stderr, "接收消息队列msgget failed with error: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    //定义发送消息队列变量
    int msgid_2;
    struct my_msg_st some_data_2;
    long int msg_to_receive_2 = 0;
    //创建消息队列
    msgid_2 = msgget((key_t) 1235, 0666 | IPC_CREAT);
    if (msgid_2 == -1) {
        fprintf(stderr, "发送消息队列msgget failed with error: %d\n", errno);
        exit(EXIT_FAILURE);
    }


    //接收网络上的传输数据
    while (1) {

        //读取消息队列的消息
        if (msgrcv(msgid, (void *) &some_data, BUFSIZ, msg_to_receive, 0) == -1) {
            fprintf(stderr, "msgrcv failed with error: %d\n", errno);
            exit(EXIT_FAILURE);
        } else {
            printf("主线程接收到的消息队列的数据为： %s\n", some_data.some_text);

            //接收到json数据进行解析
            Document d;
            d.Parse(some_data.some_text);
            Value &s = d["@table"];
            int flag = s.GetInt();

            //上传当前检测状态后 的返回信息
            if (1 == flag) {

                std::cout << "已经接收到算法发送后的返回数据" << std::endl;


            //为机型数据库添加飞机型号类别
            } else if (2 == flag) {

                string err_str;
                char *zErrMsg;
                Value &ques = d["msg"];
                if (ques.IsArray()) {

                    for (size_t i = 0; i < ques.Size(); ++i) {
                        Value &v = ques[i];
                        assert(v.IsObject());

                        const char *temp = v["CFTP"].GetString();
                        std::cout << "接收到的数据数据为1 -> " << temp << std::endl;
                        double wing_ = v["wing"].GetDouble();
                        std::cout << "接收到的数据数据为2 -> " << wing_ << std::endl;
                        double engineInner_ = v["engineInner"].GetDouble();
                        std::cout << "接收到的数据数据为3 -> " << engineInner_ << std::endl;
                        double engineOuter_ = v["engineOuter"].GetDouble();
                        std::cout << "接收到的数据数据为4 -> " << engineOuter_ << std::endl;
                        double length_ = v["length"].GetDouble();
                        std::cout << "接收到的数据数据为5 -> " << length_ << std::endl;
                        double noseheight_ = v["noseheight"].GetDouble();
                        std::cout << "接收到的数据数据为6 -> " << noseheight_ << std::endl;

                        const char *at = "INSERT INTO AIRPLANEMODEL_TAB VALUES(";

                        ostrstream oss;
                        oss << at<<"'"<<temp << "',"<<wing_<<","<<engineInner_<<","<<engineOuter_<<","<<length_<<","<<noseheight_<<");"<<'\0';
                        const char *temp_ = oss.str();
                        std::cout << temp_<<std::endl;

                        sqlite3_exec(db, temp_, 0, 0, &zErrMsg);
                        if (NULL != zErrMsg)
                        {
                            err_str += string(zErrMsg);
                            err_str += string("/");
                        }


                        selectInfoFromAIRPLANEMODEL_TAB();
                        oss.clear();
                    }



//                    char *send_buf;
//                    //回应前端信息  （写入到消息队列）
//                    if (NULL == zErrMsg)
//                    {
//                         send_buf = "{\"@table\":2,\"@src\":\"lidar\",\"error\":\"\"}";
//
//                    } else{
//
//                        const char* temp_buf =  "{\"@table\":2,\"@src\":\"lidar\",\"error\":";
//                        ostrstream err_oss;
//                        err_oss<<temp_buf<<"\""<<zErrMsg<<"\"}"<<'\0';
//                        send_buf = err_oss.str();
//                    }

                    ///////////////////////////////////////////////
                    const char *send_buf ;
                    const char *err_ch = err_str.c_str();
                    if (NULL == err_ch)
                    {
                        send_buf = "{\"@table\":2,\"@src\":\"lidar\",\"error\":\"\"}";
                    } else{

                        const char *tmp_buf = "{\"@table\":2,\"@src\":\"lidar\",\"error\":";
                        ostrstream err_oss;
                        err_oss<<tmp_buf<<"\""<<err_ch<<"\"}"<<'\0';
                        send_buf = err_oss.str();
                    }

                    /////////////////////////////////////////////




                    some_data_2.my_msg_type = 1;
                    strcpy(some_data_2.some_text, send_buf);
                    if (msgsnd(msgid_2, (void *) &some_data_2, 1024, 0) == -1)
                    {
                        fprintf(stderr, "msgsed failed\n");
                        exit(EXIT_FAILURE);
                    }else{
                        std::cout<<"主函数中已经发送消息数据～～～～～～～～～～～"<< send_buf<<std::endl;
                        continue;
                    }

                }

            //更新检测参数
            }else if(3 == flag)
            {

                string err_str;

//                std::cout<<"函数已经进来了"<<std::endl;
                Value &clustermin = d["clustermin"];
                Value &clustermax = d["clustermax"];
                Value &tolerance = d["tolerance"];
                Value &clipmin = d["clipmin"];
                Value &clipmmax = d["clipmmax"];
                Value &clipleft = d["clipleft"];
                Value &clipright = d["clipright"];

                int clustermin_ = clustermin.GetInt();
                int clustermax_ = clustermax.GetInt();
                float tolerance_ = tolerance.GetFloat();
                float clipmin_  = clipmin.GetFloat();
                float clipmax_ = clipmmax.GetFloat();
                float clipleft_ = clipleft.GetFloat();
                float clipright_ = clipright.GetFloat();



//                std::cout<<"接收到数据："<<clustermin_<<" "<<clustermax_<<" "<<tolerance_<<" "<<clipmin_<<" "<<clipmax_<<" "<<clipleft_<<" "<<clipright_<<std::endl;

                //更新参数表格
                const char *at = "UPDATE PARAMETER_TAB SET CLUSTERMIN = ";
                char *zErrMsg;
                ostrstream oss;
                oss << at<<clustermin_<<",CLUSTERMAX="<<clustermax_<<",TOLERANCE="<<tolerance_<<",CLIPMIN="<<clipmin_
                <<",CLIPMAX="<<clipmax_<<",CLIPLEFT="<<clipleft_<<",CLIPRIGHT="<< clipright_<<" WHERE ID=1"<<'\0';
                const char *temp_ = oss.str();
                std::cout << temp_<<"SQL语句"<<std::endl;
                sqlite3_exec(db, temp_, 0, 0, &zErrMsg);

                if(NULL != zErrMsg)
                {
                    std::cout<<"更新参数表格时 错误信息为:"<<zErrMsg<<std::endl;
                    err_str = string(zErrMsg) + string("/");

                }

                oss.clear();
                selectInfoFromPARAMETER_TAB();

                Value &ques = d["msg"];
                if (ques.IsArray()) {
                    for (size_t i = 0; i < ques.Size(); ++i) {
                        Value &v = ques[i];
                        assert(v.IsObject());

                        const char *model = v["CFTP"].GetString();
//                        std::cout << "接收到的数据数据为temp -> " << model << std::endl;
                        double stop1x_ = v["stop1x"].GetDouble();
//                        std::cout << "接收到的数据数据为stop1x_ -> " << stop1x_ << std::endl;
                        double stop1y_ = v["stop1y"].GetDouble();
//                        std::cout << "接收到的数据数据为stop1y_ -> " << stop1y_ << std::endl;
                        double stop2x_ = v["stop2x"].GetDouble();
//                        std::cout << "接收到的数据数据为stop2x_ -> " << stop2x_ << std::endl;
                        double stop2y_ = v["stop2y"].GetDouble();
//                        std::cout << "接收到的数据数据为stop2x_ -> " << stop2y_ << std::endl;
                        double middle1x_ = v["middle1x"].GetDouble();
//                        std::cout << "接收到的数据数据为middle1x_ -> " << middle1x_ << std::endl;
                        double middle1y_ = v["middle1y"].GetDouble();
//                        std::cout << "接收到的数据数据为middle1y_ -> " << middle1y_ << std::endl;
                        double middle2x_ = v["middle2x"].GetDouble();
//                        std::cout << "接收到的数据数据为middle1x_ -> " << middle2x_ << std::endl;
                        double middle2y_ = v["middle2y"].GetDouble();
//                        std::cout << "接收到的数据数据为middle2y_ -> " << middle2y_ << std::endl;

                        const char *at = "SELECT COUNT(*) FROM BERTH_TAB WHERE AIRPLANE_MODEL=";
                        char *zErrMsg;
                        ostrstream oss;
                        oss << at<<"'"<<model<<"'"<<";"<<'\0';
                        char *temp_ = oss.str();

                        int nrow = 0, ncolumn = 0;
                        char **azResult; //二维数组存放结果
                        sqlite3_get_table(db, temp_, &azResult, &nrow, &ncolumn, &zErrMsg);
//                        printf("row:%d column=%d \n", nrow, ncolumn);
                        printf("\nThe result of querying is : \n");
                        for (int m = 0; m < (nrow + 1) * ncolumn;m++)
                            printf("azResult[%d] = %s\n", m, azResult[m]);
                        printf("zErrMsg = %s \n", zErrMsg);
                        oss.clear();

                        if(strcmp(azResult[1],"0") == 0)         //数据库表不存在该机型，向数据库中添加
                        {

                            ostrstream add_oss;
                            const char *add = "INSERT INTO BERTH_TAB VALUES(";
                            add_oss<<add<<"'"<<model<<"',"<<stop1x_<<","<<stop1y_<<","<<stop2x_<<","<<stop2y_<<","<<middle1x_<<","<<middle1y_<<","<<middle2x_<<","<<middle2y_<<");"<<'\0';
                            char *add_sql = add_oss.str();
                            std::cout << add_sql<<std::endl;
                            sqlite3_exec(db, add_sql, 0, 0, &zErrMsg);

                            std::cout<<"该机型信息在数据库中不存在"<<std::endl;
                            if(NULL != zErrMsg)
                            {
                                err_str += string(zErrMsg);
                                err_str += string("/");
                            }


                        }else  //数据库中已存在该机型，更新数据表中该机型信息
                        {
                            std::cout<<"该机型信息在数据库中已经存在"<<std::endl;
                            ostrstream update_oss;
                            const char *update = "UPDATE BERTH_TAB SET STOP1_X=";
                            update_oss<<update<<stop1x_<<",STOP1_Y="<<stop1y_<<",STOP2_X="<<stop2x_<<",STOP2_Y="<<stop2y_
                            <<",CENTER1_X="<<middle1x_<<",CENTER1_Y="<<middle1y_<<",CENTER2_X="<<middle2x_<<",CENTER2_Y="<<middle2y_
                            <<" WHERE AIRPLANE_MODEL="<<"'"<<model<<"'"<<";"<<'\0';

                            char *update_sql = update_oss.str();
                            std::cout << update_sql<<std::endl;
                            sqlite3_exec(db, update_sql, 0, 0, &zErrMsg);

                            if(NULL != zErrMsg)
                            {
                                err_str += string(zErrMsg);
                                err_str += string("/");
                            }


                        }
                    }
                }

                const char *send_buf ;
                const char *err_ch = err_str.c_str();
                if (NULL == err_ch)
                {
                    send_buf = "{\"@table\":3,\"@src\":\"lidar\",\"error\":\"\"}";
                } else{

                    const char *tmp_buf = "{\"@table\":3,\"@src\":\"lidar\",\"error\":";
                    ostrstream err_oss;
                    err_oss<<tmp_buf<<"\""<<err_ch<<"\"}"<<'\0';
                    send_buf = err_oss.str();
                }

                //回应前端信息  （写入到消息队列）
                some_data_2.my_msg_type = 1;
                strcpy(some_data_2.some_text, send_buf);
                if (msgsnd(msgid_2, (void *) &some_data_2, 1024, 0) == -1)
                {
                    fprintf(stderr, "msgsed failed\n");
                    exit(EXIT_FAILURE);
                }else{
                    std::cout<<"主函数中已经发送消息数据～～～～～～～～～～～"<<send_buf<<std::endl;
                    continue;
                }

                //接收开始检测命令
            }else if(4 == flag)
            {

            //    std::cout<<"表4 已经进来l"<<std::endl;

                Value &workMode = d["workMode"];
                Value &workCommand = d["workCommand"];
                Value &CFTP = d["airplane"];

                workMode_ = workMode.GetInt();
                if(2 == workMode_)            // "2"：表示检测模式为：行人检测
                {
                    _aircraft_detect = false;
                }else if(1 == workMode_){     //"1":表示飞机检测
                    _aircraft_detect = true;
                }

                int workCommand_ = workCommand.GetInt();


                if(1 == workCommand_)    //1：为检测 2：为停止检测
                {

                    //获取基本参数
                    char *zErrMsg;
                    const char *sql;
                    int nrow = 0, ncolumn = 0;
                    char **azResult; //二维数组存放结果
                    sql = "SELECT * FROM PARAMETER_TAB ; ";
                    sqlite3_get_table(db, sql, &azResult, &nrow, &ncolumn, &zErrMsg);

                    if(nrow<1){
                        continue;
                        std::cout<<"行人检测参数不存在；"<<std::endl;
                    }

                    con_msg.cluster_size_min = atoi(azResult[9]);     //CLUSTERMIN
                    con_msg.cluster_size_max = atoi(azResult[10]);    //CLUSTERMAX
                    con_msg.cluster_tolerance = atof(azResult[11]) ;  //TOLERANCE
                    con_msg.clip_min_height =  atof(azResult[12]);    //CLIPMIN
                    con_msg.clip_max_height = atof(azResult[13]);     //CLIPMAX
                    con_msg.clip_left_position = atof(azResult[14]);  //CLIPLEFT
                    con_msg.clip_right_position = atof(azResult[15]); //CLIPRIGHT

                    std::cout<<con_msg.cluster_size_min<<"->"<<con_msg.cluster_size_max<<"->"<<
                    con_msg.cluster_tolerance<<"->"<<con_msg.clip_min_height<<"->"<<con_msg.clip_max_height<<"->"<<con_msg.clip_left_position<<
                    "->"<<con_msg.clip_right_position<<std::endl;


                    //获取机型参数
                    const char *model = CFTP.GetString();
                    nrow = 0;
                    ncolumn = 0;
                    const char *sql_temp  = "SELECT * FROM BERTH_TAB WHERE AIRPLANE_MODEL LIKE '";
                    ostrstream sql_oss;
                    sql_oss<<sql_temp<< model<<"';"<<'\0';
                    sql = sql_oss.str();
                    std::cout<<"查询语句是："<<sql<<std::endl;

                    sqlite3_get_table(db, sql, &azResult, &nrow, &ncolumn, &zErrMsg);

                    if(nrow<1)
                    {
                        std::cout<<"机型输入有误，请核对之！"<<std::endl;
                        continue;
                    }

                    con_msg.airplane_model = model;
                    con_msg.end_point0_x = atof(azResult[10]) ;
                    con_msg.end_point0_y = atof(azResult[11]) ;
                    con_msg.end_point1_x = atof(azResult[12]) ;
                    con_msg.end_point1_y = atof(azResult[13]) ;
                    con_msg.mild_point0_x = atof(azResult[14]);
                    con_msg.mild_point0_y = atof(azResult[15]);
                    con_msg.mild_point1_x = atof(azResult[16]);
                    con_msg.mild_point1_y = atof(azResult[17]);

                    std::cout<<con_msg.end_point0_x<<"->"<<con_msg.mild_point1_y<<std::endl;

                    _run_flag = true;    //”1“：算法开始检测
                } else
                {
                    _run_flag = false;    //”2“：算法结束检测
                }

                std::cout<<"workCommand_函数已here  = "<< workCommand_ <<"->"<<con_msg.airplane_model<<std::endl;

            }else if(6 == flag)   //添加机型的基本参数
            {
                char *zErrMsg;
                Value &v = d["msg"];
                assert(v.IsObject());


                const char *model = v["CFTP"].GetString();
                std::cout << "接收到的数据数据为1 -> " << model << std::endl;

                float wing_ =  atof(v["wing"].GetString());
                std::cout << "接收到的数据数据为2 -> " << wing_ << std::endl;

                float engineInner_ = atof(v["engineInner"].GetString());
                std::cout << "接收到的数据数据为3 -> " << engineInner_ << std::endl;

                float engineOuter_ = atof(v["engineOuter"].GetString());
                std::cout << "接收到的数据数据为4 -> " << engineOuter_ << std::endl;

                float length_ = atof(v["length"].GetString()) ;
                std::cout << "接收到的数据数据为5 -> " << length_ << std::endl;

                float noseheight_ = atof(v["noseheight"].GetString()) ;
                std::cout << "接收到的数据数据为6 -> " << noseheight_ << std::endl;

                const char *at = "INSERT INTO AIRPLANEMODEL_TAB VALUES(";
                ostrstream oss;
                oss << at<<"'"<<model << "',"<<wing_<<","<<engineInner_<<","<<engineOuter_<<","<<length_<<","<<noseheight_<<");"<<'\0';
                const char *temp_sql = oss.str();
                std::cout << temp_sql<<std::endl;

                sqlite3_exec(db, temp_sql, 0, 0, &zErrMsg);

                selectInfoFromAIRPLANEMODEL_TAB();
                oss.clear();

                //回复前端信息
                const char *send_buf ;
                if (NULL == zErrMsg)
                {
                    send_buf = "{\"@table\":6,\"@src\":\"lidar\",\"error\":\"\"}";
                } else{

                    const char *tmp_buf = "{\"@table\":6,\"@src\":\"lidar\",\"error\":";
                    ostrstream err_oss;
                    err_oss<<tmp_buf<<"\""<<zErrMsg<<"\"}"<<'\0';
                    send_buf = err_oss.str();
                }

                //回应前端信息  （写入到消息队列）
                some_data_2.my_msg_type = 1;
                strcpy(some_data_2.some_text, send_buf);
                if (msgsnd(msgid_2, (void *) &some_data_2, 1024, 0) == -1)
                {
                    fprintf(stderr, "msgsed failed\n");
                    exit(EXIT_FAILURE);
                }else{
                    std::cout<<"主函数中已经发送消息数据～～～～～～～～～～～"<<send_buf<<std::endl;
                    continue;
                }
            }else if(7 == flag)   //修改机型参数
            {
                string err_str;
                Value &ques = d["msg"];
                if (ques.IsArray()) {
                    for (size_t i = 0; i < ques.Size(); ++i) {
                        Value &v = ques[i];
                        assert(v.IsObject());

                        const char *model = v["CFTP"].GetString();
//                        std::cout << "接收到的数据数据为temp -> " << model << std::endl;
                        double wing_ = atof(v["wing"].GetString()) ;
//                        std::cout << "接收到的数据数据为stop1x_ -> " << stop1x_ << std::endl;
                        double engineInner_ = atof(v["engineInner"].GetString()) ;
//                        std::cout << "接收到的数据数据为stop1y_ -> " << stop1y_ << std::endl;
                        double engineOuter_ = atof(v["engineOuter"].GetString()) ;
//                        std::cout << "接收到的数据数据为stop2x_ -> " << stop2x_ << std::endl;
                        double length_ = atof(v["length"].GetString());
//                        std::cout << "接收到的数据数据为stop2x_ -> " << stop2y_ << std::endl;
                        double noseheight_ = atof(v["noseheight"].GetString()) ;
//                        std::cout << "接收到的数据数据为middle1x_ -> " << middle1x_ << std::endl;


                        const char *at = "SELECT COUNT(*) FROM AIRPLANEMODEL_TAB WHERE AIRPLANE_MODEL=";
                        char *zErrMsg;
                        ostrstream oss;
                        oss << at<<"'"<<model<<"'"<<";"<<'\0';
                        char *temp_ = oss.str();

                        int nrow = 0, ncolumn = 0;
                        char **azResult; //二维数组存放结果
                        sqlite3_get_table(db, temp_, &azResult, &nrow, &ncolumn, &zErrMsg);
//                        printf("row:%d column=%d \n", nrow, ncolumn);
                        printf("\nThe result of querying is : \n");
                        for (int m = 0; m < (nrow + 1) * ncolumn;m++)
                            printf("azResult[%d] = %s\n", m, azResult[m]);
                        printf("zErrMsg = %s \n", zErrMsg);
                        oss.clear();

                        if(strcmp(azResult[1],"0") == 0)         //数据库表不存在该机型，向数据库中添加
                        {
                            err_str = "The AIRPLANEMODEL_TAB table doesn't have the airplaneModel,please check!!";

                        }else  //数据库中已存在该机型，更新数据表中该机型信息
                        {
                            std::cout<<"已经找到所要修改的机型参数 AIRPLANEMODEL_TAB "<<std::endl;

                            ostrstream update_oss;
                            const char *update = "UPDATE AIRPLANEMODEL_TAB SET WING_LENGTH=";
                            update_oss<<update<<wing_<<",ENGINE_INNER="<<engineInner_<<",ENGINE_OUTER="<<engineOuter_<<",AIRPLANE_LENGTH="<<length_
                                      <<",NOSE_HEIGHT="<<noseheight_<<" WHERE AIRPLANE_MODEL="<<"'"<<model<<"'"<<";"<<'\0';

                            char *update_sql = update_oss.str();
                            std::cout << update_sql<<std::endl;
                            sqlite3_exec(db, update_sql, 0, 0, &zErrMsg);

                            if(NULL != zErrMsg)
                            {
                                err_str += string(zErrMsg);
                                err_str += string("/");
                            }

                        }
                    }
                }

                const char *send_buf ;
                const char *err_ch = err_str.c_str();
                if (NULL == err_ch)
                {
                    send_buf = "{\"@table\":7,\"@src\":\"lidar\",\"error\":\"\"}";
                } else{

                    const char *tmp_buf = "{\"@table\":7,\"@src\":\"lidar\",\"error\":";
                    ostrstream err_oss;
                    err_oss<<tmp_buf<<"\""<<err_ch<<"\"}"<<'\0';
                    send_buf = err_oss.str();
                }

                //回应前端信息  （写入到消息队列）
                some_data_2.my_msg_type = 1;
                strcpy(some_data_2.some_text, send_buf);
                if (msgsnd(msgid_2, (void *) &some_data_2, 1024, 0) == -1)
                {
                    fprintf(stderr, "msgsed failed\n");
                    exit(EXIT_FAILURE);
                }else{
                    std::cout<<"主函数中已经发送消息数据～～～～～～～～～～～"<<send_buf<<std::endl;
                    continue;
                }
            }else if(9 == flag)   //添加机型检测参数
            {

                char *zErrMsg;
                Value &v = d["msg"];
                assert(v.IsObject());


                const char *model = v["CFTP"].GetString();
                std::cout << "接收到的数据数据为1 -> " << model << std::endl;

                float stop1x =  atof(v["stop1x"].GetString());
                std::cout << "接收到的数据数据为2 -> " << stop1x << std::endl;

                float stop1y = atof(v["stop1y"].GetString());
                std::cout << "接收到的数据数据为3 -> " << stop1y << std::endl;

                float stop2x = atof(v["stop2x"].GetString());
                std::cout << "接收到的数据数据为4 -> " << stop2x << std::endl;

                float stop2y = atof(v["stop2y"].GetString()) ;
                std::cout << "接收到的数据数据为5 -> " << stop2y << std::endl;

                float middle1x = atof(v["middle1x"].GetString()) ;
                std::cout << "接收到的数据数据为6 -> " << middle1x << std::endl;

                float middle1y = atof(v["middle1y"].GetString()) ;
                std::cout << "接收到的数据数据为6 -> " << middle1y << std::endl;

                float middle2x = atof(v["middle2x"].GetString()) ;
                std::cout << "接收到的数据数据为7 -> " << middle2x << std::endl;

                float middle2y = atof(v["middle2y"].GetString()) ;
                std::cout << "接收到的数据数据为8 -> " << middle2y << std::endl;


                const char *at = "INSERT INTO BERTH_TAB VALUES(";
                ostrstream oss;
                oss << at<<"'"<<model << "',"<<stop1x<<","<<stop1y<<","<<stop2x<<","<<stop2y<<","<<middle1x<<","<<middle1y<<","<<middle2x<<","<<middle2y<<");"<<'\0';
                const char *temp_sql = oss.str();
                std::cout << temp_sql<<std::endl;

                sqlite3_exec(db, temp_sql, 0, 0, &zErrMsg);

                selectInfoFromAIRPLANEMODEL_TAB();
                oss.clear();

                //回复前端信息
                const char *send_buf ;
                if (NULL == zErrMsg)
                {
                    send_buf = "{\"@table\":9,\"@src\":\"lidar\",\"error\":\"\"}";
                } else{

                    const char *tmp_buf = "{\"@table\":9,\"@src\":\"lidar\",\"error\":";
                    ostrstream err_oss;
                    err_oss<<tmp_buf<<"\""<<zErrMsg<<"\"}"<<'\0';
                    send_buf = err_oss.str();
                }

                //回应前端信息  （写入到消息队列）
                some_data_2.my_msg_type = 1;
                strcpy(some_data_2.some_text, send_buf);
                if (msgsnd(msgid_2, (void *) &some_data_2, 1024, 0) == -1)
                {
                    fprintf(stderr, "msgsed failed\n");
                    exit(EXIT_FAILURE);
                }else{
                    std::cout<<"主函数中已经发送消息数据～～～～～～～～～～～"<<send_buf<<std::endl;
                    continue;
                }

            }else if(10 == flag)   //修改机型检测参数
            {

                string err_str;
                Value &ques = d["msg"];
                if (ques.IsArray()) {
                    for (size_t i = 0; i < ques.Size(); ++i) {
                        Value &v = ques[i];
                        assert(v.IsObject());

                        const char *model = v["CFTP"].GetString();
//                        std::cout << "接收到的数据数据为temp -> " << model << std::endl;
                        double stop1x = atof(v["stop1x"].GetString()) ;
                        std::cout << "接收到的数据数据为stop1x_ -> " << stop1x << std::endl;
                        double stop1y = atof(v["stop1y"].GetString()) ;
                        std::cout << "接收到的数据数据为stop1y_ -> " << stop1y << std::endl;
                        double stop2x = atof(v["stop2x"].GetString()) ;
                        std::cout << "接收到的数据数据为stop2x_ -> " << stop2x << std::endl;
                        double stop2y = atof(v["stop2y"].GetString());
                        std::cout << "接收到的数据数据为stop2x_ -> " << stop2y << std::endl;
                        double middle1x = atof(v["middle1x"].GetString()) ;
                        std::cout << "接收到的数据数据为middle1x_ -> " << middle1x << std::endl;
                        double middle1y = atof(v["middle1y"].GetString()) ;
                        std::cout << "接收到的数据数据为middle1y -> " << middle1y << std::endl;
                        double middle2x = atof(v["middle2x"].GetString()) ;
                        std::cout << "接收到的数据数据为middle2x -> " << middle2x << std::endl;
                        double middle2y = atof(v["middle2y"].GetString()) ;
                        std::cout << "接收到的数据数据为middle2y -> " << middle2y << std::endl;


                        const char *at = "SELECT COUNT(*) FROM BERTH_TAB WHERE AIRPLANE_MODEL=";
                        char *zErrMsg;
                        ostrstream oss;
                        oss << at<<"'"<<model<<"'"<<";"<<'\0';
                        char *temp_ = oss.str();

                        int nrow = 0, ncolumn = 0;
                        char **azResult; //二维数组存放结果
                        sqlite3_get_table(db, temp_, &azResult, &nrow, &ncolumn, &zErrMsg);
//                        printf("row:%d column=%d \n", nrow, ncolumn);
                        printf("\nThe result of querying is : \n");
                        for (int m = 0; m < (nrow + 1) * ncolumn;m++)
                            printf("azResult[%d] = %s\n", m, azResult[m]);
                        printf("zErrMsg = %s \n", zErrMsg);
                        oss.clear();

                        if(strcmp(azResult[1],"0") == 0)         //数据库表不存在该机型，向数据库中添加
                        {
                            err_str = "The Berth_TAB table doesn't have the airplaneModel,please check!!";

                        }else  //数据库中已存在该机型，更新数据表中该机型信息
                        {
                            std::cout<<"已经找到所要修改的机型参数 BERTH_TAB "<<std::endl;

                            ostrstream update_oss;
                            const char *update = "UPDATE BERTH_TAB SET STOP1_X=";
                            update_oss<<update<<stop1x<<",STOP1_Y="<<stop1y<<",STOP2_X="<<stop2x<<",STOP2_Y="<<stop2y
                                      <<",CENTER1_X="<<middle1x<<",CENTER1_Y="<<middle1y<<",CENTER2_X="<<middle2x<<",CENTER2_Y="<<middle2y<<" WHERE AIRPLANE_MODEL="<<"'"<<model<<"'"<<";"<<'\0';

                            char *update_sql = update_oss.str();
                            std::cout << update_sql<<std::endl;
                            sqlite3_exec(db, update_sql, 0, 0, &zErrMsg);

                            if(NULL != zErrMsg)
                            {
                                err_str += string(zErrMsg);
                                err_str += string("/");
                            }

                        }
                    }
                }

                const char *send_buf ;
                const char *err_ch = err_str.c_str();
                if (NULL == err_ch)
                {
                    send_buf = "{\"@table\":10,\"@src\":\"lidar\",\"error\":\"\"}";
                } else{

                    const char *tmp_buf = "{\"@table\":10,\"@src\":\"lidar\",\"error\":";
                    ostrstream err_oss;
                    err_oss<<tmp_buf<<"\""<<err_ch<<"\"}"<<'\0';
                    send_buf = err_oss.str();
                }

                //回应前端信息  （写入到消息队列）
                some_data_2.my_msg_type = 1;
                strcpy(some_data_2.some_text, send_buf);
                if (msgsnd(msgid_2, (void *) &some_data_2, 1024, 0) == -1)
                {
                    fprintf(stderr, "msgsed failed\n");
                    exit(EXIT_FAILURE);
                }else{
                    std::cout<<"主函数中已经发送消息数据～～～～～～～～～～～"<<send_buf<<std::endl;
                    continue;
                }

            }else if(12 == flag)     //修改行人检测参数
            {
                string err_str;
                std::cout<<"函数已经进来了"<<std::endl;
                Value &clustermin = d["clustermin"];
                Value &clustermax = d["clustermax"];
                Value &tolerance = d["tolerance"];
                Value &clipmin = d["clipmin"];
                Value &clipmmax = d["clipmmax"];     //
                Value &clipleft = d["clipleft"];
                Value &clipright = d["clipright"];

                int clustermin_ = atoi(clustermin.GetString());
                int clustermax_ = atoi(clustermax.GetString());
                float tolerance_ = atof(tolerance.GetString());
                float clipmin_  = atof(clipmin.GetString()) ;
                float clipmax_ = atof(clipmmax.GetString());
                float clipleft_ = atof(clipleft.GetString());
                float clipright_ = atof(clipright.GetString());

                std::cout<<"接收到数据："<<clustermin_<<" "<<clustermax_<<" "<<tolerance_<<" "<<clipmin_<<" "<<clipmax_<<" "<<clipleft_<<" "<<clipright_<<std::endl;

                //更新参数表格
                const char *at = "UPDATE PARAMETER_TAB SET CLUSTERMIN = ";
                char *zErrMsg;
                ostrstream oss;
                oss << at<<clustermin_<<",CLUSTERMAX="<<clustermax_<<",TOLERANCE="<<tolerance_<<",CLIPMIN="<<clipmin_
                    <<",CLIPMAX="<<clipmax_<<",CLIPLEFT="<<clipleft_<<",CLIPRIGHT="<< clipright_<<" WHERE ID=1"<<'\0';
                const char *temp_ = oss.str();
                std::cout << temp_<<"SQL语句"<<std::endl;
                sqlite3_exec(db, temp_, 0, 0, &zErrMsg);

                if(NULL != zErrMsg)
                {
                    std::cout<<"更新行人检测参数表格时 错误信息为:"<<zErrMsg<<std::endl;
                    err_str = string(zErrMsg) + string("/");

                }

                oss.clear();
                selectInfoFromPARAMETER_TAB();

                const char *send_buf ;
                const char *err_ch = err_str.c_str();
                if (NULL == err_ch)
                {
                    send_buf = "{\"@table\":12,\"@src\":\"lidar\",\"error\":\"\"}";
                } else{

                    const char *tmp_buf = "{\"@table\":12,\"@src\":\"lidar\",\"error\":";
                    ostrstream err_oss;
                    err_oss<<tmp_buf<<"\""<<err_ch<<"\"}"<<'\0';
                    send_buf = err_oss.str();
                }

                //回应前端信息  （写入到消息队列）
                some_data_2.my_msg_type = 1;
                strcpy(some_data_2.some_text, send_buf);
                if (msgsnd(msgid_2, (void *) &some_data_2, 1024, 0) == -1)
                {
                    fprintf(stderr, "msgsed failed\n");
                    exit(EXIT_FAILURE);
                }else{
                    std::cout<<"主函数中已经发送消息数据～～～～～～～～～～～"<<send_buf<<std::endl;
                    continue;
                }

            }



        }


        usleep(500 * 1000);

    }
    std::cout << "main, and InformationDisplayHandler execute concurrently...\n";
//  InformationDisplayHandler.join();
    PclDealthreadHandler.join();
//  recvHandler.join();
    std::cout << "function completed.\n";

//  recvLocalSocket_handle.join();
    sendLocalSOcket_handle.join();
    return 0;
}