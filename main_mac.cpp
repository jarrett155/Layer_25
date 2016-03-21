/*
 * Copyright: CampusLink project
 * 
 * This is a tester program for WLAN MAC layer.
 * 
 * Author: Farzad Hessar
*/
#include "mac_handler.h"
#include "dcf_handler.h"
#include "phy_handler.h"
#include "stdio.h"
#include "string.h"
#include <iostream>
#include <ctime>
#include "time.h"
#include <signal.h>
#include <queue>
#include <map>
#include <vector>
#include "pthread.h"
#include <iterator>


#define DATA_SIZE 2000
#define TEST_MCS_INDEX 0
#define ACK_TIMEOUT_MS 500

#include "stdarg.h"
#define default_log_file "mac_handler_tester.log"

//#define log_report(...) do { log_write("" __VA_ARGS__); } while(0)
#define log_report(...) printf("" __VA_ARGS__)
void log_write(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    FILE* fp = fopen(default_log_file, "a+");
    vfprintf(fp, format, args);
    //fprintf(fp, "dummy_log\n");
    va_end(args);
    fclose(fp);
}

struct NC_Client_Data {
    std::queue<RX_datagram> * received_data;
    std::map<int,std::vector<uint8_t> > *NC_data_remembered;
    int seq_to_ack;
    bool needs_to_NC_ACK;
};

struct IP_NC_Frame
{
    int ip_src, ip_dst;
    int seq_num_from;
    std::vector<uint8_t> data;
};


struct NC_Dest
{
    std::queue<IP_NC_Frame> to_send;
    bool waiting_for_ack;
    int seq_need_ack;
    IP_NC_Frame data_needs_ack;
    int size;
    bool must_retransmit;
    pthread_t timeout_id;
    int ip_dst;
    // to add ack nums that can do NC
};

class AP_Handler
{
    std::vector<NC_Dest> destinations;
    std::map<int,int> IP_to_index_map;
    int current_destination;
    int current_size;
    int frames_to_send;
    IP_NC_Frame to_code;
    int pop_nc_dest;
    int nc_dest_popped;
    bool retransmit;
    int start;


public:

    AP_Handler();

    bool AP_Has_Next();

    void received_Frame(int seq_num, int ip_dst, int ip_src, std::vector<uint8_t> data);

    bool has_Matching_Frame(IP_NC_Frame frame_to_match);

    IP_NC_Frame get_NC_Frame();

    IP_NC_Frame get_Next();

    void set_NC_Ack(int seq_num, int ip_dest);

    void pop_Next_Frame();

    void ack_Frame(int src_ip, int seq_num);

};

void Test_DCF_A();
void Test_DCF_B();
void Test_DCF_A_SEND();
void Test_DCF_B_ACK();
void RTT_estimation(bool is_sender);
void Test_MAC_A();
void Test_MAC_B();
void Test_MAC_A_rate_adapt();
void Test_MAC_B_rate_adapt();
void Detection_delay_estimation(bool is_sender);
void Test_Relay_AA();
void Test_Relay_BB();
void Test_Relay_CC();
void Test_NC_AA();
void Test_NC_BB();
void Test_NC_AP_00(bool use_NC);

int main(int argc, char *argv[]){
    int desiredTest  = -1;

    while(desiredTest<1 || desiredTest>18){
        std::cout << "Select desired application: \n";
        std::cout << "   1) DCF test MACaddr = 0xAA, send to 0xBB\n";
        std::cout << "   2) DCF test MACaddr = 0xBB, send to 0xAA\n";
        std::cout << "   3) DCF test MACaddr = 0xAA, sending DATA\n";
        std::cout << "   4) DCF test MACaddr = 0xBB, sending ACK\n";
        std::cout << "   5) RTT estimation at distinct rate and size for STA A, sending\n";
        std::cout << "   6) RTT estimation at distinct rate and size for STA B, ACKing. 6 first,then 5\n";
        std::cout << "   7) MAC test STA A, send to STA B\n";
        std::cout << "   8) MAC test STA B, send to STA A\n";
        std::cout << "   9) MAC test STA A with rate adaptation, send to STA B\n";
        std::cout << "   10) MAC test STA B with rate adaptation, send to STA A\n";          
        std::cout << "   11) Detection delay estimation at MCS 0 with distince size for STA A, sending\n";          
        std::cout << "   12) Detection delay estimation for STA B, acking, 12 first, then 11\n";        
        std::cout << "   13) relay test AA\n";
        std::cout << "   14) relay test BB\n";        
        std::cout << "   15) relay test CC\n";
        std::cout << "   16) NC test client AA\n";
        std::cout << "   17) NC test client BB\n";
        std::cout << "   18) NC test AP CC\n";
        
        std::cin >> desiredTest;
    }
    
    switch(desiredTest){
        case 1:
            Test_DCF_A();
            break;
	    case 2:
            Test_DCF_B();
        case 3:
            Test_DCF_A_SEND();
            break;
        case 4:
            Test_DCF_B_ACK();
            break;
        case 5:
            RTT_estimation(true);
            break;
        case 6:
            RTT_estimation(false);
            break;
        case 7:
            Test_MAC_A();
            break;
        case 8:
            Test_MAC_B();
            break;
        case 9:
            Test_MAC_A_rate_adapt();
            break;
        case 10:
            Test_MAC_B_rate_adapt();            
            break;
        case 11:
            Detection_delay_estimation(true);
            break;
        case 12:
            Detection_delay_estimation(false);
            break;
        case 13:
            Test_Relay_AA();
            break;
        case 14:
            Test_Relay_BB();
            break;
        case 15:
            Test_Relay_CC();
            break;
        case 16:
            Test_NC_AA();
            break;
        case 17:
            Test_NC_BB();
            break;
        case 18:
            Test_NC_AP_00(true);
            break;
        default:
            break;
    }
    return 0;
}




//AP HANDLER CODE. SHOULD ADD TO OWN FILE BUT CMAKE

void* ack_Timeout(void *must_retransmit)
{
    
    usleep(1000*ACK_TIMEOUT_MS);
    *((bool*)must_retransmit) = true;
    return NULL;
}

AP_Handler::AP_Handler()
{
    retransmit = false;
    current_destination = 0;
    current_size = 0;
    frames_to_send = 0;
    nc_dest_popped = 1;
}

// returns true if the AP has a frame to send otherwise returns false
bool AP_Handler::AP_Has_Next()
{
    return frames_to_send;
}

// adds a received frame to its destination queue
void AP_Handler::received_Frame(int seq_num, int ip_dst, int ip_src, std::vector<uint8_t> data)
{
    // check if we have seen this ip before
    if (IP_to_index_map.count(ip_dst))
    {
        //std::cout << "\ndest exists " << ip_dst <<" out of " << IP_to_index_map.size() << "\n\n\n";
        int index_of_dst = IP_to_index_map[ip_dst];
        IP_NC_Frame frame_received;
        frame_received.ip_src = ip_src;
        frame_received.ip_dst = ip_dst;
        frame_received.data = data;
        frame_received.seq_num_from = seq_num;
        destinations[index_of_dst].to_send.push(frame_received);
        destinations[index_of_dst].size ++;
        frames_to_send ++;

    }
    // if not add a new destination node to AP handler
    else
    {
        std::cout << "\n\n\ncreating new dest " << ip_dst << "\n\n\n\n";
        
        NC_Dest new_dest;
        new_dest.waiting_for_ack = false;
        new_dest.seq_need_ack = 0;
        new_dest.size = 1;
        new_dest.must_retransmit = false;
        IP_to_index_map[ip_dst] = current_size;
        IP_NC_Frame frame_received;
        frame_received.ip_src = ip_src;
        frame_received.ip_dst = ip_dst;
        frame_received.data = data;
        frame_received.seq_num_from = seq_num;
        new_dest.to_send.push(frame_received);
        destinations.push_back(new_dest);
        current_size ++;
        frames_to_send ++;
    }
}

// Find if a frame can be network coded with frame_to_match
bool AP_Handler::has_Matching_Frame(IP_NC_Frame frame_to_match)
{
    //std::cout << "checking for nc\n";
    if (destinations[current_destination].waiting_for_ack)
    {
        //std::cout << "no NC, waiting for ack\n";
        return false;
    }
    else
    {
        int nc_dest = frame_to_match.ip_src;
        if(IP_to_index_map.count(nc_dest))
        {
            int nc_dest_index = IP_to_index_map[nc_dest];
            if(destinations[nc_dest_index].size > 0)
            {
                if( !destinations[nc_dest_index].waiting_for_ack && destinations[nc_dest_index].to_send.front().ip_src == frame_to_match.ip_dst)
                {
                    //std::cout << "\n\n\n\nwooooo NC"<< current_destination <<" " << nc_dest <<"\n\n\n\n\n";

                    nc_dest_popped = 0;
                    pop_nc_dest = nc_dest_index;
                    to_code = destinations[nc_dest_index].to_send.front();
                    //std::cout << "found code frame\n\n";
                    return true;
                }
            }
        }
    }
    return false;

}

// Get the frame that will be network coded with frame_to_match.
// Only call after has_Matching_Frame returns true
IP_NC_Frame AP_Handler::get_NC_Frame()
{
    return to_code;
}

// Get the next frame that will be sent by the AP
IP_NC_Frame AP_Handler::get_Next()
{
    //std::cout << "setting start";
    start = current_destination;

    do 
    {
        current_destination = (current_destination + 1) % current_size;
        if(destinations[current_destination].must_retransmit)
        {
            retransmit = true;
            return destinations[current_destination].data_needs_ack;
        }
    } while(destinations[current_destination].size < 1 && current_destination != start);
    if (destinations[current_destination].size > 0)
    {
        //std::cout << "returning frame to send\n";
        //std::cout << "current dest is " << current_destination;
        //std::cout << " \ncurrent size is " << current_size;
        //std::cout << " current frames to send is " << frames_to_send;
        //std::cout << " current dest size " << destinations[current_destination].to_send.size();
        
        
        return destinations[current_destination].to_send.front();
    }
    else
    {
        //need to fix this
        std::cout<< "bad news gonna crash";
        IP_NC_Frame hi;
        return hi;
    }
}

// Set the ack for frames used in NC. Must be called once for each of the
// destinations in an NC frame.
void AP_Handler::set_NC_Ack(int seq_num, int ip_to)
{
    int index_of_dst = IP_to_index_map[ip_to];
    destinations[index_of_dst].waiting_for_ack = true;
    destinations[index_of_dst].data_needs_ack = destinations[index_of_dst].to_send.front();
    destinations[index_of_dst].seq_need_ack = seq_num;
    destinations[index_of_dst].must_retransmit = false;
    void *timer_arg = &(destinations[index_of_dst].must_retransmit);
    pthread_create(&destinations[index_of_dst].timeout_id, NULL, ack_Timeout, timer_arg);

}

// Used to clear out a sent frame once it is done sending.
// Must be called after a frame is sent.
void AP_Handler::pop_Next_Frame()
{
    if(!retransmit)
    {
        //std::cout<< "frame popped " <<frames_to_send - 1<< " frames left \n";
        destinations[current_destination].to_send.pop();
        destinations[current_destination].size --;
        frames_to_send --;
    }
    else
        retransmit = false;
    if(nc_dest_popped == 0)
    {
        //std::cout << "\n" << pop_nc_dest << " dest popped for nc its size " << destinations[pop_nc_dest].to_send.size()
        //        << "other size" << destinations[pop_nc_dest].size << "\n";
        destinations[pop_nc_dest].to_send.pop();
        destinations[pop_nc_dest].size --;
        nc_dest_popped = 1;
        frames_to_send --;
    }
    //std::cout <<"done popping\n";
}

// This function is used to ack a frame that was previously sent as an NC frame.
// Call this when an NC ack is received. 
void AP_Handler::ack_Frame(int src_ip, int seq_num)
{
    int index_of_dst = IP_to_index_map[src_ip];
    if (seq_num == destinations[index_of_dst].seq_need_ack)
    {
        destinations[index_of_dst].waiting_for_ack = false;
        pthread_cancel(destinations[index_of_dst].timeout_id);
        destinations[index_of_dst].must_retransmit = false;
    }
}

// END AP HANDLER





//##########################################################################
//used for blocking callback
void dcf_rx_helper(void* context, RX_mac_frame * frame)
{
    //printf("receive a DCF frame\n");
    return;
} 

/**
 * General test for DCF, intended for multi-stations sends frames at the same 
 * time. This function set the MAC address to be 11:22:33:44:55:66
 */
void Test_DCF_A(){
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    
    myAddr[0] = 0x11;
    myAddr[1] = 0x22;
    myAddr[2] = 0x33;
    myAddr[3] = 0x44;
    myAddr[4] = 0x55;
    myAddr[5] = 0x66;
    
    toAddr[0] = 0x77;
    toAddr[1] = 0x88;
    toAddr[2] = 0x99;
    toAddr[3] = 0xAA;
    toAddr[4] = 0xBB;
    toAddr[5] = 0xCC;
       
    frame_80211 tx_frame;
    uint16_t frame_ctrl =0x0000;
    frame_ctrl=((frame_ctrl ) << 4 | SUBTYPE_DATA ) << 4 | DATA_FRAME;
    tx_frame.header.frame_ctrl = frame_ctrl;
    tx_frame.header.dur_id = ACK_PHY_TIME  + MAC_RX_PROC_TIME;
    memcpy(tx_frame.header.addr1,toAddr,MAC_ADDR_LENGTH);
    memcpy(tx_frame.header.addr2,myAddr,MAC_ADDR_LENGTH);
    tx_frame.header.seq_ctl = 0x0000;
    memcpy(tx_frame.buffer_and_fcs, data, sizeof(data));
     
    std::queue<Queue_frame> queue;  
    Dcf_handler handle;
    
    void dcf_rx_helper(void* context, RX_mac_frame * frame);
    handle.dcf_setRxCallback_context(dcf_rx_helper, NULL);
    
    handle.set_myadx(myAddr); 
    handle.start_dcf();   
    int total_frame = 0, success_frame = 0;
    while(1){
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);
//        std::cout << std::endl << "\t\t\t[Host] START new frame" << std::endl;        
        status =  handle.send_frame((uint8_t*) &tx_frame, sizeof(frame_header_80211_DATA) + sizeof(data),&queue);
        total_frame++;
        switch(status){
        case 0:
            success_frame++;
            if(success_frame%100 ==0)
                std::cout << std::endl << "\t\t\t[Host] Success! " << (double)success_frame/ total_frame*100 << "%" << std::endl;
            break;
        case -1:  
 //           std::cout << std::endl << "\t\t\t[Host] Failure. discard." << std::endl;
            break;
        case -2:
 //           std::cout << std::endl << "\t\t\t[Host] Failure! (busy)" << std::endl;
            break;
        }     
    }
}


//##########################################################################
/**
 * General test for DCF, intended for multi-stations sends frames at the same 
 * time. This function set the MAC address to be 77:88:99:AA:BB:CC
 */
void Test_DCF_B(){
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    
    toAddr[0] = 0x11;
    toAddr[1] = 0x22;
    toAddr[2] = 0x33;
    toAddr[3] = 0x44;
    toAddr[4] = 0x55;
    toAddr[5] = 0x66;
    
    myAddr[0] = 0x77;
    myAddr[1] = 0x88;
    myAddr[2] = 0x99;
    myAddr[3] = 0xAA;
    myAddr[4] = 0xBB;
    myAddr[5] = 0xCC;
       
    frame_80211 tx_frame;
    uint16_t frame_ctrl =0x0000;
    frame_ctrl=((frame_ctrl ) << 4 | SUBTYPE_DATA ) << 4 | DATA_FRAME;
    tx_frame.header.frame_ctrl = frame_ctrl;
    tx_frame.header.dur_id = ACK_PHY_TIME  + MAC_RX_PROC_TIME;
    memcpy(tx_frame.header.addr1,toAddr,MAC_ADDR_LENGTH);
    memcpy(tx_frame.header.addr2,myAddr,MAC_ADDR_LENGTH);
    tx_frame.header.seq_ctl = 0x0000;
    memcpy(tx_frame.buffer_and_fcs, data, sizeof(data));
     
    std::queue<Queue_frame> queue;  
    Dcf_handler handle;
    
    void dcf_rx_helper(void* context, RX_mac_frame * frame);
    handle.dcf_setRxCallback_context(dcf_rx_helper, NULL);
    
    handle.set_myadx(myAddr); 
    handle.start_dcf();    
    int total_frame = 0, success_frame = 0;
    while(1){
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);
  //      std::cout << std::endl << "\t\t\t[Host] START new frame" << std::endl;        
        status =  handle.send_frame((uint8_t*) &tx_frame, sizeof(frame_header_80211_DATA) + sizeof(data),&queue);
        total_frame++;
        switch(status){
        case 0:
            success_frame++;
            if(success_frame%100 == 0)
                std::cout << std::endl << "\t\t\t[Host] Success! " << (double)success_frame/total_frame*100 << "%" << std::endl;
            break;
        case -1:  
            std::cout << std::endl << "\t\t\t[Host] Failure. discard." << std::endl;
            break;
        case -2:
            std::cout << std::endl << "\t\t\t[Host] Failure! (busy)" << std::endl;
            break;   
        }          
    }
}


//##########################################################################
/**
 * This task is used with Test_DCF_B_ACK(). Test_DCF_A_SEND() is sending DATA 
 * only. The Test_DCF_B_ACK() receive DATA and send ACK. Test_DCF_B_ACK() 
 * does not send DATA. Test_DCF_A_SEND() does not send ACK. 
 */
void Test_DCF_A_SEND(){
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    
    myAddr[0] = 0xAA;
    toAddr[0] = 0xBB;
       
    frame_80211 tx_frame;
    uint16_t frame_ctrl =0x0000;
    frame_ctrl=( (frame_ctrl ) << 4 | SUBTYPE_DATA ) << 4 | DATA_FRAME;
    tx_frame.header.frame_ctrl = frame_ctrl;
    tx_frame.header.dur_id = 0;
    memcpy(tx_frame.header.addr1,toAddr,MAC_ADDR_LENGTH);
    memcpy(tx_frame.header.addr2,myAddr,MAC_ADDR_LENGTH);
    tx_frame.header.seq_ctl = 0x0000;
    memcpy(tx_frame.buffer_and_fcs, data, sizeof(data));
     
    std::queue<Queue_frame> queue; 
    Dcf_handler handle;
    
    void dcf_rx_helper(void* context, RX_mac_frame * frame);
    handle.dcf_setRxCallback_context(dcf_rx_helper, NULL);
    
    handle.set_myadx(myAddr); 
    handle.start_dcf();    
        
    int total = 0, success = 0;
    struct timespec dcf_start, dcf_now,dcf_operating;
    dcf_operating.tv_nsec = 0;
    dcf_operating.tv_nsec = 0;
    double dcf_operating_time = 0;
    while(1){        
        clock_gettime(CLOCK_MONOTONIC, &dcf_start);
        status =  handle.send_frame((uint8_t*) &tx_frame, sizeof(frame_header_80211_DATA) + sizeof(data),&queue);
        clock_gettime(CLOCK_MONOTONIC, &dcf_now);
        dcf_operating.tv_sec = dcf_now.tv_sec - dcf_start.tv_sec;
        dcf_operating.tv_nsec = dcf_now.tv_nsec - dcf_start.tv_nsec;                
        while (dcf_operating.tv_nsec > 1000000000) 
        {            
            dcf_operating.tv_sec++;
            dcf_operating.tv_nsec -= 1000000000;
        }
        while (dcf_operating.tv_nsec < 0) 
        {
            dcf_operating.tv_sec--;
            dcf_operating.tv_nsec += 1000000000;
        }   
        dcf_operating_time += dcf_operating.tv_sec + dcf_operating.tv_nsec/1000000000.0; 
                                    
        total++;
        switch(status){
        case 0:
            success++;
            if(success%100 ==0)
            {
                printf("\t\t\t#%d frame DCF time: %-7.3fs, suc frm %d, rate %.2f%%\n\n",total,dcf_operating_time,success,100*success/(double)total); 
                std::cout << std::endl << "\t\t\t[Host] Success! "<< (double)success/total*100 << "%" << std::endl;            
            }
            break;
        case -1:  
            std::cout << std::endl << "\t\t\t[Host] Failure. discard." << std::endl;
            break;
        case -2:
            std::cout << std::endl << "\t\t\t[Host] Failure! (busy)" << std::endl;
            break;
        }     
    }
}


//##########################################################################
/** this task is used with Test_DCF_A_SEND()*/
void Test_DCF_B_ACK(){
    uint8_t myAddr[6];
    
    memset(myAddr, 0, 6);
    
    myAddr[0] = 0xBB;         
        
    Dcf_handler handle;
    void dcf_rx_helper(void* context, RX_mac_frame * frame);
    handle.dcf_setRxCallback_context(dcf_rx_helper, NULL);
    handle.set_myadx(myAddr); 
    handle.start_dcf();        
    while(1){  
        usleep(1000000);
    }
}


//##########################################################################
/** 
 * this task is used for RTT measuring which accesses physical layer directly
 * without contacting with DCF. The RTT is measured for eight modes from 0 to 
 * 7, and the testing data size can be specified below, in the array called
 * "data_size". This task outputs measured RTT in to "RTT_estimation.log" for 
 * each specific data size and mode. There is a MATLAB processing file in the 
 * drop-box which helps to analyze data. 
 */
#define RTT_LOG_ENABLED
#ifdef RTT_LOG_ENABLED
#define RTT_log_report(...) printf("" __VA_ARGS__);
#else
#define RTT_log_report(...) do {} while(0);
#endif

struct timespec time_stampA;
struct timespec time_stampB;
struct tx_data data_packet;
struct tx_data ACK_packet;
phy_handler handle_RTT;

uint8_t mode_stage = 0;
int total_size = 10;
uint32_t data_size[10]={16,256,512,752,1000,1248,1500,1748,2000,2248};
int size_index = 0;
int TX_times = 300;
bool have_ack = false;
timespec RTT_nano_stamp,RTT_nano_start;
timespec time_step_wait;
char RTT_filename[50] = "RTT_estimation.log";
int data_status = -100;

void RTT_estimation(bool is_sender)
{
    fclose(fopen(RTT_filename,"w"));   
    
    ACK_packet.length = 14;
    ACK_packet.mode = IEEE80211TVWSModes(0); 
    
    memset(ACK_packet.buffer, 0, ACK_packet.length-4);
    memset(ACK_packet.buffer+ACK_packet.length-4, 0, 4);   //last 4 bytes left for 0
    
    time_step_wait.tv_sec = 0;
    time_step_wait.tv_nsec = 50000;
    
    int status = handle_RTT.start_phy(1);
    if (!status)
    {
        RTT_log_report("phy start success\n");
    }
    else
    {
        RTT_log_report("phy start fail\n");
    }
      
    if (is_sender)
    {        
        void RTT_ACK_receiver_rx_callback(rx_data* packet_rx);
        handle_RTT.setRxCallback(&RTT_ACK_receiver_rx_callback);
        RTT_log_report("SET ACK receiver callback DONE\n");

        for (mode_stage = 0; mode_stage <=7; mode_stage++)
        {
            data_packet.mode = IEEE80211TVWSModes(mode_stage);
            for (size_index = 0; size_index<total_size; size_index++)
            {   
                data_packet.length = data_size[size_index];                
                memset(data_packet.buffer, 0, data_packet.length);
                
                data_packet.mode = IEEE80211TVWSModes(mode_stage);
                
                unsigned long timeout_ms = 30;
                
                for (int i=0; i<TX_times; i++)
                {
                    RTT_log_report("Mode:%d  Size:%d  TX:%d\n",mode_stage,data_size[size_index], i);
                    data_packet.buffer[10] = i/256;
                    data_packet.buffer[11] = i%256;
                    
                    clock_gettime(CLOCK_MONOTONIC, &time_stampA);
                    
                    data_status = handle_RTT.sendPacket(&data_packet);
                    RTT_log_report("data send status %d", data_status);
                    unsigned long nTime;
                    clock_gettime(CLOCK_MONOTONIC,&RTT_nano_start);
                    bool time_in_flag = true;  
                    while(!have_ack && time_in_flag)
                    {
                        clock_nanosleep(CLOCK_MONOTONIC, 0, &time_step_wait,NULL);
                        clock_gettime(CLOCK_MONOTONIC,&RTT_nano_stamp);
                        nTime = (RTT_nano_stamp.tv_sec - RTT_nano_start.tv_sec)*NANO_PER_SEC + RTT_nano_stamp.tv_nsec - RTT_nano_start.tv_nsec;
                        if (nTime >= timeout_ms*1000000)
                        {
                            time_in_flag = false;
                        }     
                    }

                    if(!have_ack)
                    {                        
                        RTT_log_report("\t\t\t failure\n");
                    }
                    else
                    {
                        RTT_log_report("\t\t\t success\n");
                        have_ack = false; 
                    }
                }
            }
        }  
    }
    else
    {       
        void RTT_ACK_sender_rx_callback(rx_data*);
        handle_RTT.setRxCallback(&RTT_ACK_sender_rx_callback);
        RTT_log_report("SET ACK Sender callback DONE\n");
        while(1){usleep(100000);}
    }        
    return;
}


uint8_t pre_size_index = 0;
struct timespec RTT_end_time, RTT_ACK_start;
long proc_dur = 0;
void RTT_ACK_receiver_rx_callback(rx_data* packet_rx)
{      
    if(packet_rx->isFrameDetected && packet_rx->CRC_valid)
    {
        clock_gettime(CLOCK_MONOTONIC, &time_stampB);                
        unsigned int RTT = ((time_stampB.tv_sec - time_stampA.tv_sec)*1000000000 + time_stampB.tv_nsec) - time_stampA.tv_nsec;
        FILE* stream = fopen(RTT_filename,"a+");

        uint8_t curr_size_index = packet_rx->buffer[6];
        
        if (curr_size_index != pre_size_index)
        {                        
            pre_size_index = curr_size_index;                
            fprintf (stream, "#\n");              
        }
                             
        fprintf (stream, "%d\n", RTT/1000);  
        fclose(stream);
        have_ack = true;          
    }
}

timespec ack_start, ack_end;
long timespend = 0;
uint8_t s_index = 0;
uint8_t next_index = 1;
int ack_status = -100;
void RTT_ACK_sender_rx_callback(rx_data* packet_rx){
    if(packet_rx->isFrameDetected && packet_rx->CRC_valid){ 
        //put frame size index in the buffer
        RTT_log_report("Frame Size %d   BB:%d\n",packet_rx->length,packet_rx->BB_amplitude);
        if(packet_rx->length == data_size[next_index])
        {
            RTT_log_report("\t\t new %d old %d\n",packet_rx->length,data_size[s_index]);
            s_index++;
            s_index = s_index%(total_size);         
            next_index = (s_index+1)%total_size;
        }
        ACK_packet.buffer[6] = s_index;                
        //put frame RATE in the buffer
        ACK_packet.buffer[7] = packet_rx->mode;
        //put frame ID in the ACK packet as well
        ACK_packet.buffer[8] = packet_rx->buffer[10];
        ACK_packet.buffer[9] = packet_rx->buffer[11];
        ack_status = handle_RTT.sendPacket(&ACK_packet);                   
        RTT_log_report("ack status %d\n",ack_status);
    }
}

//##########################################################################
/**
 * General Mac layer tests, intended for use of multi station scenario, the 
 * MAC address is 0x112233445566
 */
void Test_MAC_A()
{
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    
    myAddr[0] = 0x11;
    myAddr[1] = 0x22;
    myAddr[2] = 0x33;
    myAddr[3] = 0x44;
    myAddr[4] = 0x55;
    myAddr[5] = 0x66;
    
    toAddr[0] = 0x77;
    toAddr[1] = 0x88;
    toAddr[2] = 0x99;
    toAddr[3] = 0xAA;
    toAddr[4] = 0xBB;
    toAddr[5] = 0xCC;
       
    Mac_handler handler;
    void mac_STA_receiver(RX_datagram * mac_data);
    handler.mac_setRxCallback(mac_STA_receiver);
    handler.set_myadx(myAddr);
    handler.set_PHY_TX_MCS(TEST_MCS_INDEX);
    
    handler.start_mac();
           
    int total_frame = 0, success_frame = 0;
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    while(1){
        data[0] = total_frame%256;
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);        
  //      std::cout << std::endl << "            [Host] START new frame" << std::endl;         
        status =  handler.send_frame(data,toAddr,sizeof(data));
        total_frame++;
        switch(status){
        case 0:
            success_frame++;
 //           std::cout << std::endl << "        [Host] Success! " << success_frame << "/" << total_frame << std::endl;
            break;
        case -1:  
            std::cout << std::endl << "            [Host] Failure. discard." << std::endl;
            break;
        case -2:
  //          std::cout << std::endl << "            [Host] Failure! (busy)" << std::endl;
            break;   
        }          
    }   
}


//##########################################################################
/**
 * General Mac layer tests, intended for use of multi station scenario, the 
 * MAC address is 0x778899aabbcc
 */
void Test_MAC_B()
{
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    
    toAddr[0] = 0x11;
    toAddr[1] = 0x22;
    toAddr[2] = 0x33;
    toAddr[3] = 0x44;
    toAddr[4] = 0x55;
    toAddr[5] = 0x66;
    
    myAddr[0] = 0x77;
    myAddr[1] = 0x88;
    myAddr[2] = 0x99;
    myAddr[3] = 0xAA;
    myAddr[4] = 0xBB;
    myAddr[5] = 0xCC;
    
    Mac_handler handler;
    void mac_STA_receiver(RX_datagram * mac_data);
    handler.mac_setRxCallback(mac_STA_receiver);
    handler.set_myadx(myAddr);        
    //handler.use_FPGA_callback_in_DCF(true);
    handler.set_PHY_TX_MCS(TEST_MCS_INDEX);
    handler.start_mac();

    int total_frame = 0, success_frame = 0;
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    while(1){
        data[0] = total_frame%256;
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);        
 //       std::cout << std::endl << "            [Host] START new frame" << std::endl;                
        status =  handler.send_frame(data,toAddr,sizeof(data));
        total_frame++;
        switch(status){
        case 0:
            success_frame++;
 //           std::cout << std::endl << "        [Host] Success! " << success_frame << "/" << total_frame << std::endl;
            break;
        case -1:  
            std::cout << std::endl << "            [Host] Failure. discard." << std::endl;
            break;
        case -2:
//            std::cout << std::endl << "            [Host] Failure! (busy)" << std::endl;
            break;   
        }          
    }
}

extern struct timeval rx_micro_start,rx_micro_end;
extern unsigned int rx_timer;

int total_rx_frame =0;
//##########################################################################
/** function to be called by MAC to deliver data*/
void mac_STA_receiver(RX_datagram * mac_data)
{
    // process data
    //printf("        receive size %d\n",mac_data->data_size); 
    printf ("%d\n", total_rx_frame++);
    //printf("        receive data %s\n",mac_data->data); 
    return;
}

//##########################################################################
/**
 * General Mac layer tests with rate adaptation, intended for use of multi 
 * station scenario, the MAC address is 0x112233445566
 */
void Test_MAC_A_rate_adapt()
{
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    
    myAddr[0] = 0x11;
    myAddr[1] = 0x22;
    myAddr[2] = 0x33;
    myAddr[3] = 0x44;
    myAddr[4] = 0x55;
    myAddr[5] = 0x66;
    
    toAddr[0] = 0x77;
    toAddr[1] = 0x88;
    toAddr[2] = 0x99;
    toAddr[3] = 0xAA;
    toAddr[4] = 0xBB;
    toAddr[5] = 0xCC;
       
    Mac_handler handler;
    void mac_STA_receiver(RX_datagram * mac_data);
    handler.mac_setRxCallback(mac_STA_receiver);
    handler.set_myadx(myAddr);  
    handler.enable_rate_adaptation();  
    //handler.use_FPGA_callback_in_DCF(true);    
    handler.start_mac();

    
    int total_frame = 0, success_frame = 0;
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    while(1){
        data[0] = total_frame%256;
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);        
 //       std::cout << std::endl << "            [Host] START new frame" << std::endl;         
        status =  handler.send_frame(data,toAddr,sizeof(data));
        total_frame++;
        switch(status){
        case 0:
            success_frame++;
   //         std::cout << std::endl << "        [Host] Success! " << success_frame << "/" << total_frame << std::endl;
            break;
        case -1:  
            std::cout << std::endl << "            [Host] Failure. discard." << std::endl;
            break;
        case -2:
  //          std::cout << std::endl << "            [Host] Failure! (busy)" << std::endl;
            break;   
        }          
    }   
}


//##########################################################################
/**
 * General Mac layer tests with rate adaptation, intended for use of multi 
 * station scenario, the MAC address is 0x778899aabbcc
 */
void Test_MAC_B_rate_adapt()
{
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    
    toAddr[0] = 0x11;
    toAddr[1] = 0x22;
    toAddr[2] = 0x33;
    toAddr[3] = 0x44;
    toAddr[4] = 0x55;
    toAddr[5] = 0x66;
    
    myAddr[0] = 0x77;
    myAddr[1] = 0x88;
    myAddr[2] = 0x99;
    myAddr[3] = 0xAA;
    myAddr[4] = 0xBB;
    myAddr[5] = 0xCC;
    
    Mac_handler handler;
    void mac_STA_receiver(RX_datagram * mac_data);
    handler.mac_setRxCallback(mac_STA_receiver);
    handler.set_myadx(myAddr);    
    handler.enable_rate_adaptation();
    //handler.use_FPGA_callback_in_DCF(true);
    
    handler.start_mac();

    
    int total_frame = 0, success_frame = 0;
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    while(1){
        data[0] = total_frame%256;
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);        
  //      std::cout << std::endl << "            [Host] START new frame" << std::endl;                
        status =  handler.send_frame(data,toAddr,sizeof(data));
        total_frame++;
        switch(status){
        case 0:
            success_frame++;
  //          std::cout << std::endl << "        [Host] Success! " << success_frame << "/" << total_frame << std::endl;
            break;
        case -1:  
            std::cout << std::endl << "            [Host] Failure. discard." << std::endl;
            break;
        case -2:
 //           std::cout << std::endl << "            [Host] Failure! (busy)" << std::endl;
            break;   
        }          
    }
}



//#############################################################################
/**
 * Detection estimation function used to measure the amount of time from STA A
 * sending until STA B detecting. It is also used for other measurements like 
 * physical layer ACK time, RTT. This test runs under MCS 0-7, MSC 0 represents 
 * the worst case detection delay, and is the longest physical layer ACK time.
 * After running this test, a log file with name "detection_estimation.log" is 
 * generated under directory "host/output". The analysis will be carried out 
 * by a Matlab script located under directory "host/utilities/wlan/tools", with
 * a name "Detection_estimation.m". More information about measured time 
 * parameters can be found inside the Matlab script. While running this test, 
 * make sure use cable with attenuator as transmission medium and most of data 
 * should get success, 300 packets are used to measure the time parameter, on 
 * my machine it gets 295 or higher good transmission.
 */ 
struct timespec data_start_time;
struct timespec ack_start_time;
struct timespec ack_received_time;

struct timespec sync_detected_time;
struct timespec header_detected_time;

unsigned int detect_to_receive_delay;
//struct tx_data data_packet; defined above
//struct tx_data ACK_packet;
phy_handler handle_detection;


char detection_filename[50] = "detection_estimation.log";
void Detection_delay_estimation(bool is_sender)
{
    fclose(fopen(detection_filename,"w"));    
    void TestSyncHdr_SyncCallback(void*);
    void TestSyncHdr_HDRCallback(void*, int, int, bool);    
    handle_detection.setRxSyncCallback_context(TestSyncHdr_SyncCallback, NULL);
    handle_detection.setRxHdrCallback_context(TestSyncHdr_HDRCallback, NULL);
    
    ACK_packet.length = 14;
    ACK_packet.mode = IEEE80211TVWSModes(0); 
    
    memset(ACK_packet.buffer, 0, ACK_packet.length-4);
    memset(ACK_packet.buffer+ACK_packet.length-4, 0, 4);   //last 4 bytes left for 0
    
    time_step_wait.tv_sec = 0;
    time_step_wait.tv_nsec = 50000;
    
    int status = handle_detection.start_phy(1);
    if (!status)
    {
        RTT_log_report("phy start success\n");
    }
    else
    {
        RTT_log_report("phy start fail\n");
    }
    
    if (is_sender)
    {        
        void Detection_ACK_receiver_rx_callback(rx_data* packet_rx);
        handle_detection.setRxCallback(&Detection_ACK_receiver_rx_callback);
        RTT_log_report("SET ACK receiver callback DONE\n");
        
        for (mode_stage = 0; mode_stage <=7; mode_stage++)
        {
            data_packet.mode = IEEE80211TVWSModes(mode_stage);
            for (size_index = 0; size_index<total_size; size_index++)
            {   
                data_packet.length = data_size[size_index];                
                memset(data_packet.buffer, 0, data_packet.length);

                data_packet.mode = IEEE80211TVWSModes(mode_stage);

                unsigned long timeout_ms = 30;

                for (int i=0; i<TX_times; i++)
                {
                    RTT_log_report("Mode:%d  Size:%d  TX:%d\n",mode_stage,data_size[size_index], i);
                    data_packet.buffer[10] = i/256;
                    data_packet.buffer[11] = i%256;

                    clock_gettime(CLOCK_MONOTONIC, &data_start_time);

                    data_status = handle_detection.sendPacket(&data_packet);
                    RTT_log_report("data send status %d", data_status);
                    unsigned long nTime;
                    clock_gettime(CLOCK_MONOTONIC,&RTT_nano_start);                

                    bool time_in_flag = true;  
                    while(!have_ack && time_in_flag)
                    {
                        clock_nanosleep(CLOCK_MONOTONIC, 0, &time_step_wait,NULL);
                        clock_gettime(CLOCK_MONOTONIC,&RTT_nano_stamp);
                        nTime = (RTT_nano_stamp.tv_sec - RTT_nano_start.tv_sec)*NANO_PER_SEC + RTT_nano_stamp.tv_nsec - RTT_nano_start.tv_nsec;
                        if (nTime >= timeout_ms*1000000)
                        {
                            time_in_flag = false;
                        }     
                    }                                

                    if(!have_ack)
                    {                        
                        RTT_log_report("\t\t\t failure\n");
                    }
                    else
                    {
                        RTT_log_report("\t\t\t success\n");
                        have_ack = false; 
                    }
                }
                FILE* stream = fopen(detection_filename,"a+");                
                fprintf (stream, "#\n");              
                fclose(stream);
            }
        }
    }
    else
    {       
        void Detection_ACK_sender_rx_callback(rx_data*);
        handle_detection.setRxCallback(&Detection_ACK_sender_rx_callback);
        RTT_log_report("SET ACK Sender callback DONE\n");
        while(1){usleep(100000);}
    }        
    return;
}

/** when DATA sender receives an ACK*/
void Detection_ACK_receiver_rx_callback(rx_data* packet_rx)
{      
    if(packet_rx->isFrameDetected && packet_rx->CRC_valid)
    {
        clock_gettime(CLOCK_MONOTONIC, &ack_received_time);                
        unsigned int RTT = ((ack_received_time.tv_sec - data_start_time.tv_sec)*1000000000 + ack_received_time.tv_nsec) - data_start_time.tv_nsec;
        FILE* stream = fopen(detection_filename,"a+");       
                
        fprintf (stream, "%ld.%09ld\n", data_start_time.tv_sec, data_start_time.tv_nsec);                         
        fprintf (stream, "%-8d\n", RTT/1000);  
        
        
        fclose(stream);
        have_ack = true;          
    }
}    

/** when receiving a packet, this function responses an ACK*/
void Detection_ACK_sender_rx_callback(rx_data* packet_rx){
   


    if(packet_rx->isFrameDetected && packet_rx->CRC_valid){ 
        clock_gettime(CLOCK_MONOTONIC, &ack_start_time);
        //put frame size index in the buffer
        RTT_log_report("Frame Size %d   BB:%d\n",packet_rx->length,packet_rx->BB_amplitude);
        if(packet_rx->length == data_size[next_index])
        {
            RTT_log_report("\t\t new %d old %d\n",packet_rx->length,data_size[s_index]);
            s_index++;
            s_index = s_index%(total_size);         
            next_index = (s_index+1)%total_size;
        }
        ACK_packet.buffer[6] = s_index;                
        //put frame RATE in the buffer
        ACK_packet.buffer[7] = packet_rx->mode;
        //put frame ID in the ACK packet as well
        ACK_packet.buffer[8] = packet_rx->buffer[10];
        ACK_packet.buffer[9] = packet_rx->buffer[11];
        ack_status = handle_detection.sendPacket(&ACK_packet); 
        detect_to_receive_delay = ((ack_start_time.tv_sec - sync_detected_time.tv_sec)*1000000000 + ack_start_time.tv_nsec) - sync_detected_time.tv_nsec;
        FILE* stream = fopen(detection_filename,"a+");
        fprintf (stream, "%d\n%ld.%09ld\n", detect_to_receive_delay/NANO_PER_MICRO ,ack_start_time.tv_sec, ack_start_time.tv_nsec); 
        fclose(stream); 
        
        RTT_log_report("ack status %d\n",ack_status);
    }
}
//when receiving a sync from physical layer
void TestSyncHdr_SyncCallback(void* context){
    clock_gettime(CLOCK_MONOTONIC, &sync_detected_time);    
    //sync_signal++;
}

//when receiving a header from physical layer
void TestSyncHdr_HDRCallback(void* context, int mcs, int length, bool crc_valid){
    ;
}

void Test_Relay_AA()
{    
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    
    myAddr[0] = 0xAA;
    myAddr[1] = 0xAA;
    myAddr[2] = 0xAA;
    myAddr[3] = 0xAA;
    myAddr[4] = 0xAA;
    myAddr[5] = 0xAA;
           
    Mac_handler handler;
    void mac_STA_receiver(RX_datagram * mac_data);
    handler.mac_setRxCallback(mac_STA_receiver);
    handler.set_myadx(myAddr);
    handler.set_PHY_TX_MCS(TEST_MCS_INDEX);
    handler.set_AP(true);
    handler.start_mac();
               
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    while(1){
        sleep(1)          ;
    } 
    return;
}


void Test_Relay_BB()
{
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t apAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    
    data[0] = 'F';
    data[1] = 'r';
    data[2] = 'o';
    data[3] = 'm';
    data[4] = ' ';
    data[5] = 'B';
    data[6] = 'B';
    data[7] = '\0';            
    
    myAddr[0] = 0xBB;
    myAddr[1] = 0xBB;
    myAddr[2] = 0xBB;
    myAddr[3] = 0xBB;
    myAddr[4] = 0xBB;
    myAddr[5] = 0xBB;
    
    toAddr[0] = 0xCC;
    toAddr[1] = 0xCC;
    toAddr[2] = 0xCC;
    toAddr[3] = 0xCC;
    toAddr[4] = 0xCC;
    toAddr[5] = 0xCC;
    
    apAddr[0] = 0xAA;
    apAddr[1] = 0xAA;
    apAddr[2] = 0xAA;
    apAddr[3] = 0xAA;
    apAddr[4] = 0xAA;
    apAddr[5] = 0xAA;
           
    Mac_handler handler;
    void mac_STA_receiver(RX_datagram * mac_data);
    handler.mac_setRxCallback(mac_STA_receiver);
    handler.set_myadx(myAddr);
    handler.set_PHY_TX_MCS(TEST_MCS_INDEX);
    handler.set_AP(false);
    handler.start_mac();
           
    int total_frame = 0, success_frame = 0;
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    while(1){
        //data[0] = total_frame%256;
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);        
  //      std::cout << std::endl << "            [Host] START new frame" << std::endl;         
        status =  handler.send_frame(data,toAddr, apAddr,sizeof(data));        
        total_frame++;
        switch(status){
        case 0:
            success_frame++;
 //           std::cout << std::endl << "        [Host] Success! " << success_frame << "/" << total_frame << std::endl;
            break;
        case -1:  
            std::cout << std::endl << "            [Host] Failure. discard." << std::endl;
            break;
        case -2:
  //          std::cout << std::endl << "            [Host] Failure! (busy)" << std::endl;
            break;   
        }          
    }
}


void Test_Relay_CC()
{
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t apAddr[6];
    uint8_t data[DATA_SIZE];
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));
    data[0] = 'F';
    data[1] = 'r';
    data[2] = 'o';
    data[3] = 'm';
    data[4] = ' ';
    data[5] = 'C';
    data[6] = 'C';
    data[7] = '\0';    
    
    myAddr[0] = 0xCC;
    myAddr[1] = 0xCC;
    myAddr[2] = 0xCC;
    myAddr[3] = 0xCC;
    myAddr[4] = 0xCC;
    myAddr[5] = 0xCC;
    
    toAddr[0] = 0xBB;
    toAddr[1] = 0xBB;
    toAddr[2] = 0xBB;
    toAddr[3] = 0xBB;
    toAddr[4] = 0xBB;
    toAddr[5] = 0xBB;
    
    apAddr[0] = 0xAA;
    apAddr[1] = 0xAA;
    apAddr[2] = 0xAA;
    apAddr[3] = 0xAA;
    apAddr[4] = 0xAA;
    apAddr[5] = 0xAA;
           
    Mac_handler handler;
    void mac_STA_receiver(RX_datagram * mac_data);
    handler.mac_setRxCallback(mac_STA_receiver);
    handler.set_myadx(myAddr);
    handler.set_PHY_TX_MCS(TEST_MCS_INDEX);
    handler.set_AP(false);
    handler.start_mac();
           
    int total_frame = 0, success_frame = 0;
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    while(1){
        //data[0] = total_frame%256;
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);        
  //      std::cout << std::endl << "            [Host] START new frame" << std::endl;         
        status =  handler.send_frame(data,toAddr, apAddr,sizeof(data));        
        total_frame++;
        switch(status){
        case 0:
            success_frame++;
 //           std::cout << std::endl << "        [Host] Success! " << success_frame << "/" << total_frame << std::endl;
            break;
        case -1:  
            std::cout << std::endl << "            [Host] Failure. discard." << std::endl;
            break;
        case -2:
  //          std::cout << std::endl << "            [Host] Failure! (busy)" << std::endl;
            break;   
        }          
    }
    
    
}

//##########################################################################
/** function to be called by MAC to deliver data*/

void mac_NC_receiver(RX_datagram * mac_data)
{
    // process data
    //printf ("%d\n", total_rx_frame++);
    //printf("        receive data %s\n", mac_data->data + 16); 
    return;
}

void* client_Rx_thread(void *client_data_arg)
{
    NC_Client_Data * client_data = client_data_arg;
    std::queue<RX_datagram> * received_data = client_data->received_data;
    std::map<int,std::vector<uint8_t> > * NC_data_remembered = client_data->NC_data_remembered;
    while(1)
    {
        if (!received_data->empty())
        {
            RX_datagram NC_frame = received_data->front();
            received_data->pop();
            if(NC_frame.data[6] == 0xAA && NC_frame.data[7] == 0xAA)
            {
                int seq_num1 = 256*NC_frame.data[8] + NC_frame.data[9];
                int seq_num2 = 256*NC_frame.data[10] + NC_frame.data[11];
                int ip1 = 256*256*256*NC_frame.data[12] + 256*256*NC_frame.data[13] + 256*NC_frame.data[14] + NC_frame.data[15];
                int ip2 = 256*256*256*NC_frame.data[16] + 256*256*NC_frame.data[17] + 256*NC_frame.data[18] + NC_frame.data[19];
                if( ip1 == myip)
                {
                    if (NC_data_remembered->count(seq_num1))
                    {
                        client_data->seq_to_ack = seq_num1;
                        client_data->needs_to_NC_ACK = true;
                        std::vector<uint8_t> remembered_data = (*NC_data_remembered)[seq_num1];
                        NC_data_remembered->erase(seq_num1);
                        std::vector<uint8_t> decoded_data;
                        for (int i = 0; i <= 19; i++)
                        {
                            decoded_data.push_back(remembered_data[i]^NC_frame.data[20+i]);
                        }
                        std::copy(decoded_data.begin(), decoded_data.end(), std::ostream_iterator<uint8_t>(std::cout, ""));
                    }
                    

                }
                else if(ip2 == myip)
                {
                    if (NC_data_remembered->count(seq_num2))
                    {
                        client_data->seq_to_ack = seq_num2;
                        client_data->needs_to_NC_ACK = true;
                        std::vector<uint8_t> remembered_data = (*NC_data_remembered)[seq_num2];
                        NC_data_remembered->erase(seq_num2);
                        std::vector<uint8_t> decoded_data;
                        for (int i = 0; i <= 19; i++)
                        {
                            decoded_data.push_back(remembered_data[i]^NC_frame.data[20+i]);
                        }
                        std::copy(decoded_data.begin(), decoded_data.end(), std::ostream_iterator<char>(std::cout, ""));
                    }
                }
            }
            else
            {
                printf("      receive data %s\n", NC_frame.data); 
            }
        }
    }
}


void Test_NC_AA()
{
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t apAddr[6];
    uint8_t data[DATA_SIZE];
    NC_Client_Data client_data;
    int LLC_Header_Size = 8;
    int NC_L25_Header_Size = 2;
    int L3_Header_Size = 8;
    client_data.seq_to_ack = 0;
    client_data.needs_to_NC_ACK = false;
    // const int max_frames_rememebered = 1000;
    int data_start;
    int L3_start;
    int myip = 1;
    std::map<int,std::vector<uint8_t> > NC_data_remembered;
    //std::map<int,std::vector<uint8_t> >::iterator it;
    client_data.NC_data_remembered = &NC_data_remembered;
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));  
    
    myAddr[0] = 0xAA;
    myAddr[1] = 0xAA;
    myAddr[2] = 0xAA;
    myAddr[3] = 0xAA;
    myAddr[4] = 0xAA;
    myAddr[5] = 0xAA;
    
    toAddr[0] = 0x00;
    toAddr[1] = 0x00;
    toAddr[2] = 0x00;
    toAddr[3] = 0x00;
    toAddr[4] = 0x00;
    toAddr[5] = 0x00;
    
    apAddr[0] = 0xCC;
    apAddr[1] = 0xCC;
    apAddr[2] = 0xCC;
    apAddr[3] = 0xCC;
    apAddr[4] = 0xCC;
    apAddr[5] = 0xCC;
           
    Mac_handler handler;
    std::queue<RX_datagram> received_data;
    void mac_NC_receiver(RX_datagram * mac_data);
    handler.mac_setRxCallback(mac_NC_receiver);
    handler.set_frame_queue(&received_data);
    client_data.received_data = &received_data;
    handler.set_myadx(myAddr);
    handler.set_PHY_TX_MCS(TEST_MCS_INDEX);
    handler.set_AP(false);
    handler.start_mac();

    int total_frame = 0, success_frame = 0;
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    pthread_t Rx_thread;

    pthread_create(&Rx_thread, NULL, client_Rx_thread, (void*) &client_data);

    //  Do experiment here by sending frames
    while(1){
    /*
        if (!received_data.empty())
        {
            RX_datagram NC_frame = received_data.front();
            received_data.pop();
            if(NC_frame.data[6] == 0xAA && NC_frame.data[7] == 0xAA)
            {
                int seq_num1 = 256*NC_frame.data[8] + NC_frame.data[9];
                int seq_num2 = 256*NC_frame.data[10] + NC_frame.data[11];
                int ip1 = 256*256*256*NC_frame.data[12] + 256*256*NC_frame.data[13] + 256*NC_frame.data[14] + NC_frame.data[15];
                int ip2 = 256*256*256*NC_frame.data[16] + 256*256*NC_frame.data[17] + 256*NC_frame.data[18] + NC_frame.data[19];
                if( ip1 == myip)
                {
                    it = NC_data_remembered.find(seq_num1);
                    if (it != NC_data_remembered.end())
                    {
                        seq_to_ack = seq_num1;
                        needs_to_NC_ACK = true;
                        std::vector<uint8_t> remembered_data = NC_data_remembered[seq_num1];
                        NC_data_remembered.erase(seq_num1);
                        std::vector<uint8_t> decoded_data;
                        for (int i = 0; i <= 19; i++)
                        {
                            decoded_data.push_back(remembered_data[i]^NC_frame.data[20+i]);
                        }
                        std::copy(decoded_data.begin(), decoded_data.end(), std::ostream_iterator<uint8_t>(std::cout, ""));
                    }
                    

                }
                else if(ip2 == myip)
                {
                    it = NC_data_remembered.find(seq_num2);
                    if (it != NC_data_remembered.end())
                    {
                        seq_to_ack = seq_num2;
                        needs_to_NC_ACK = true;
                        std::vector<uint8_t> remembered_data = NC_data_remembered[seq_num2];
                        NC_data_remembered.erase(seq_num2);
                        std::vector<uint8_t> decoded_data;
                        for (int i = 0; i <= 19; i++)
                        {
                            decoded_data.push_back(remembered_data[i]^NC_frame.data[20+i]);
                        }
                        std::copy(decoded_data.begin(), decoded_data.end(), std::ostream_iterator<char>(std::cout, ""));
                    }
                }
            }
            else
            {
                printf("      receive data %s\n", NC_frame.data); 
            }
        }
        */
        //data[0] = total_frame%256;
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);        
  //    std::cout << std::endl << "            [Host] START new frame" << std::endl;
        if(needs_to_NC_ACK)
        {
            //set NC header and LLC header
            data_start = LLC_Header_Size + NC_L25_Header_Size + L3_Header_Size;
            L3_start = data_start - L3_Header_Size;

            // LLC Header for layer 2.5
            data[0] = 0xAA;
            data[1] = 0XAA;
            data[2] = 0X03;
            data[3] = 0XAA;
            data[4] = 0XAA;
            data[5] = 0XAA;
            data[6] = 0XAA;
            data[7] = 0XAA;

            // NC Header for ACK
            data[8] = seq_to_ack/256;
            data[9] = seq_to_ack;

            // source ip addr
            data[L3_start + 0] = 0;
            data[L3_start + 1] = 0;
            data[L3_start + 2] = 0;
            data[L3_start + 3] = 1;

            // dest ip addr
            data[L3_start + 4] = 0;
            data[L3_start + 5] = 0;
            data[L3_start + 6] = 0;
            data[L3_start + 7] = 2;

            data[data_start + 0] = 'F';
            data[data_start + 1] = 'r';
            data[data_start + 2] = 'o';
            data[data_start + 3] = 'm';
            data[data_start + 4] = ' ';
            data[data_start + 5] = 'A';
            data[data_start + 6] = 'A';
            data[data_start + 7] = 'A';
            data[data_start + 10] = '\0';
            needs_to_NC_ACK = false;  
        }
        else
        {
            //set LLC header to standard val
            data_start = LLC_Header_Size + L3_Header_Size;
            L3_start = data_start - L3_Header_Size;

            //LLC Header for normal
            data[0] = 0xAA;
            data[1] = 0XAA;
            data[2] = 0X03;
            data[3] = 0X00;
            data[4] = 0X00;
            data[5] = 0X00;
            data[6] = 0X08;
            data[7] = 0X00;


            // source ip addr
            data[L3_start + 0] = 0;
            data[L3_start + 1] = 0;
            data[L3_start + 2] = 0;
            data[L3_start + 3] = 1;

            // dest ip addr
            data[L3_start + 4] = 0;
            data[L3_start + 5] = 0;
            data[L3_start + 6] = 0;
            data[L3_start + 7] = 2;


            data[data_start + 0] = 'F';
            data[data_start + 1] = 'r';
            data[data_start + 2] = 'o';
            data[data_start + 3] = 'm';
            data[data_start + 4] = ' ';
            data[data_start + 5] = 'A';
            data[data_start + 6] = 'A';
            data[data_start + 6] = ' ';
            data[data_start + 10] = '\0';  

        }

        data[data_start + 8] = success_frame / 256;
        data[data_start + 9] = success_frame;        
        status = handler.send_frame(data,toAddr, apAddr,sizeof(data));
        
        total_frame++;



        switch(status){
        case 0:
        {
            success_frame++;
 //           std::cout << std::endl << "        [Host] Success! " << success_frame << "/" << total_frame << std::endl;
            int seq_num_sent = handler.get_seq_num();
            std::vector<uint8_t> to_remember(data + L3_start, data + data_start + 11);
            NC_data_remembered.insert(std::pair<int,std::vector<uint8_t> >(seq_num_sent, to_remember));
            break;
        }
        case -1:  
            std::cout << std::endl << "            [Host] Failure. discard." << std::endl;
            break;
        case -2:
  //          std::cout << std::endl << "            [Host] Failure! (busy)" << std::endl;
            break;   
        }          
    }
    
    
}

void Test_NC_BB()
{
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t apAddr[6];
    uint8_t data[DATA_SIZE];
    NC_Client_Data client_data;
    int LLC_Header_Size = 8;
    int NC_L25_Header_Size = 2;
    int L3_Header_Size = 8;
    client_data.seq_to_ack = 0;
    client_data.needs_to_NC_ACK = false;
    // const int max_frames_rememebered = 1000;
    int data_start;
    int L3_start;
    int myip = 2;
    int seq_to_ack = 0;
    bool needs_to_NC_ACK = false;
    std::map<int,std::vector<uint8_t> > NC_data_remembered;
    //std::map<int,std::vector<uint8_t> >::iterator it;
    client_data.NC_data_remembered = &NC_data_remembered;
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));  
    
    myAddr[0] = 0xBB;
    myAddr[1] = 0xBB;
    myAddr[2] = 0xBB;
    myAddr[3] = 0xBB;
    myAddr[4] = 0xBB;
    myAddr[5] = 0xBB;
    
    toAddr[0] = 0x00;
    toAddr[1] = 0x00;
    toAddr[2] = 0X00;
    toAddr[3] = 0x00;
    toAddr[4] = 0x00;
    toAddr[5] = 0x00;
    
    apAddr[0] = 0x00;
    apAddr[1] = 0x00;
    apAddr[2] = 0x00;
    apAddr[3] = 0x00;
    apAddr[4] = 0x00;
    apAddr[5] = 0x00;
           
    Mac_handler handler;
    std::queue<RX_datagram> received_data;
    void mac_NC_receiver(RX_datagram * mac_data);
    handler.set_frame_queue(&received_data);
    handler.mac_setRxCallback(mac_NC_receiver);
    handler.set_myadx(myAddr);
    handler.set_PHY_TX_MCS(TEST_MCS_INDEX);
    handler.set_AP(false);
    handler.start_mac();

    int total_frame = 0, success_frame = 0;
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    pthread_t Rx_thread;

    pthread_create(&Rx_thread, NULL, client_Rx_thread, (void*) &client_data);

    //  Do experiment here by sending frames
    while(1){
        /*
        if (!received_data.empty())
        {
            RX_datagram NC_frame = received_data.front();
            received_data.pop();
            if(NC_frame.data[6] == 0xAA && NC_frame.data[7] == 0xAA)
            {
                int seq_num1 = 256*NC_frame.data[8] + NC_frame.data[9];
                int seq_num2 = 256*NC_frame.data[10] + NC_frame.data[11];
                int ip1 = 256*256*256*NC_frame.data[12] + 256*256*NC_frame.data[13] + 256*NC_frame.data[14] + NC_frame.data[15];
                int ip2 = 256*256*256*NC_frame.data[16] + 256*256*NC_frame.data[17] + 256*NC_frame.data[18] + NC_frame.data[19];
                if( ip1 == myip)
                {
                    it = NC_data_remembered.find(seq_num1);
                    if (it != NC_data_remembered.end())
                    {
                        seq_to_ack = seq_num1;
                        needs_to_NC_ACK = true;
                        std::vector<uint8_t> remembered_data = NC_data_remembered[seq_num1];
                        NC_data_remembered.erase(seq_num1);
                        std::vector<uint8_t> decoded_data;
                        for (int i = 0; i <= 19; i++)
                        {
                            decoded_data.push_back(remembered_data[i]^NC_frame.data[20+i]);
                        }
                        std::copy(decoded_data.begin(), decoded_data.end(), std::ostream_iterator<uint8_t>(std::cout, ""));
                    }
                    

                }
                else if(ip2 == myip)
                {
                    it = NC_data_remembered.find(seq_num2);
                    if (it != NC_data_remembered.end())
                    {
                        seq_to_ack = seq_num2;
                        needs_to_NC_ACK = true;
                        std::vector<uint8_t> remembered_data = NC_data_remembered[seq_num2];
                        NC_data_remembered.erase(seq_num2);
                        std::vector<uint8_t> decoded_data;
                        for (int i = 0; i <= 19; i++)
                        {
                            decoded_data.push_back(remembered_data[i]^NC_frame.data[20+i]);
                        }
                        std::copy(decoded_data.begin(), decoded_data.end(), std::ostream_iterator<char>(std::cout, ""));
                    }
                }
            }
            else
            {
                printf("      receive data %s\n", NC_frame.data);
                
            }
        }
        */
        //data[0] = total_frame%256;
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);        
  //    std::cout << std::endl << "            [Host] START new frame" << std::endl;
        if(needs_to_NC_ACK)
        {
            //set NC header and LLC header
            data_start = LLC_Header_Size + NC_L25_Header_Size + L3_Header_Size;
            L3_start = data_start - L3_Header_Size;

            // LLC Header for layer 2.5
            data[0] = 0xAA;
            data[1] = 0XAA;
            data[2] = 0X03;
            data[3] = 0XAA;
            data[4] = 0XAA;
            data[5] = 0XAA;
            data[6] = 0XAA;
            data[7] = 0XAA;

            // NC Header for ACK
            data[8] = seq_to_ack/256;
            data[9] = seq_to_ack;

            // source ip addr
            data[L3_start + 0] = 0;
            data[L3_start + 1] = 0;
            data[L3_start + 2] = 0;
            data[L3_start + 3] = 2;

            // dest ip addr
            data[L3_start + 4] = 0;
            data[L3_start + 5] = 0;
            data[L3_start + 6] = 0;
            data[L3_start + 7] = 1;

            data[data_start + 0] = 'F';
            data[data_start + 1] = 'r';
            data[data_start + 2] = 'o';
            data[data_start + 3] = 'm';
            data[data_start + 4] = ' ';
            data[data_start + 5] = 'B';
            data[data_start + 6] = 'B';
            data[data_start + 7] = ' ';
            data[data_start + 10] = '\0';
            needs_to_NC_ACK = false;  
        }
        else
        {
            //set LLC header to standard val
            data_start = LLC_Header_Size + L3_Header_Size;
            L3_start = data_start - L3_Header_Size;

            //LLC Header for normal
            data[0] = 0xAA;
            data[1] = 0XAA;
            data[2] = 0X03;
            data[3] = 0X00;
            data[4] = 0X00;
            data[5] = 0X00;
            data[6] = 0X08;
            data[7] = 0X00;


            // source ip addr
            data[L3_start + 0] = 0;
            data[L3_start + 1] = 0;
            data[L3_start + 2] = 0;
            data[L3_start + 3] = 2;

            // dest ip addr
            data[L3_start + 4] = 0;
            data[L3_start + 5] = 0;
            data[L3_start + 6] = 0;
            data[L3_start + 7] = 1;


            data[data_start + 0] = 'F';
            data[data_start + 1] = 'r';
            data[data_start + 2] = 'o';
            data[data_start + 3] = 'm';
            data[data_start + 4] = ' ';
            data[data_start + 5] = 'A';
            data[data_start + 6] = 'A';
            data[data_start + 6] = ' ';
            data[data_start + 10] = '\0';  

        }

        data[data_start + 8] = success_frame / 256;
        data[data_start + 9] = success_frame;        
        status = handler.send_frame(data,toAddr, apAddr,sizeof(data));
        
        total_frame++;



        switch(status){
        case 0:
        {
            success_frame++;
 //           std::cout << std::endl << "        [Host] Success! " << success_frame << "/" << total_frame << std::endl;
            int seq_num_sent = handler.get_seq_num();
            std::vector<uint8_t> to_remember(data + L3_start, data + data_start + 11);
            NC_data_remembered.insert(std::pair<int,std::vector<uint8_t> >(seq_num_sent, to_remember));
            break;
        }
        case -1:  
            std::cout << std::endl << "            [Host] Failure. discard." << std::endl;
            break;
        case -2:
  //          std::cout << std::endl << "            [Host] Failure! (busy)" << std::endl;
            break;   
        }          
    }
    
    
}

void Test_NC_AP_00(bool use_NC)
{    
    int status;
    uint8_t myAddr[6];
    uint8_t toAddr[6];
    uint8_t apAddr[6];
    uint8_t data[DATA_SIZE];
    int count = 0;
    int frames_uncoded = 0;
    int frames_coded = 0;
    
    int LLC_Header_Size = 8;
    int NC_L25_Header_Size = 12;
    int L3_Header_Size = 8;

    std::map<int,std::vector<uint8_t> > IP_to_MAC_map;

    int data_start;
    int L3_start;
    
    memset(myAddr, 0, 6);
    memset(toAddr, 0, 6);
    memset(data,0,sizeof(data));

    myAddr[0] = 0x00;
    myAddr[1] = 0x00;
    myAddr[2] = 0x00;
    myAddr[3] = 0x00;
    myAddr[4] = 0x00;
    myAddr[5] = 0x00;

    toAddr[0] = 0xFF;
    toAddr[1] = 0xFF;
    toAddr[2] = 0xFF;
    toAddr[3] = 0xFF;
    toAddr[4] = 0xFF;
    toAddr[5] = 0xFF;

    std::vector<uint8_t> MAC_addr(toAddr, toAddr + sizeof(toAddr) / sizeof(uint8_t));
    IP_to_MAC_map.insert(std::pair<int,std::vector<uint8_t> >(0, MAC_addr));

    
    MAC_addr[0] = 0xAA;
    MAC_addr[1] = 0xAA;
    MAC_addr[2] = 0xAA;
    MAC_addr[3] = 0xAA;
    MAC_addr[4] = 0xAA;
    MAC_addr[5] = 0xAA;

    IP_to_MAC_map.insert(std::pair<int,std::vector<uint8_t> >(1, MAC_addr));

    MAC_addr[0] = 0xBB;
    MAC_addr[1] = 0xBB;
    MAC_addr[2] = 0xBB;
    MAC_addr[3] = 0xBB;
    MAC_addr[4] = 0xBB;
    MAC_addr[5] = 0xBB;
    
    IP_to_MAC_map.insert(std::pair<int,std::vector<uint8_t> >(2, MAC_addr));
    
    apAddr[0] = 0x00;
    apAddr[1] = 0x00;
    apAddr[2] = 0x00;
    apAddr[3] = 0x00;
    apAddr[4] = 0X00;
    apAddr[5] = 0x00;
           
    Mac_handler handler;
    std::queue<RX_datagram> received_data;
    void mac_NC_receiver(RX_datagram * mac_data);

    AP_Handler ap_handle;
    handler.set_frame_queue(&received_data);
    handler.mac_setRxCallback(mac_NC_receiver);
    handler.set_myadx(myAddr);
    handler.set_PHY_TX_MCS(TEST_MCS_INDEX);
    handler.set_AP(false);
    handler.start_mac();
    IP_NC_Frame sending_frame;

    int total_frame = 0, success_frame = 0;
    std::cout << std::endl << "            [Host] is transmitting." << std::endl;
    

    //  Do experiment here by sending frames
    while(1){
        count = count + 1;
        if (!received_data.empty())
        {
            RX_datagram NC_frame = received_data.front();
            received_data.pop();
            if(NC_frame.data[6] == 0xAA && NC_frame.data[7] == 0xAA)
            {
                std::cout << "receivd NC ack";
                int seq_num_ack = 256*NC_frame.data[8] + NC_frame.data[9];
                int ip_src = 256*256*256*NC_frame.data[10] + 256*256*NC_frame.data[11] + 256*NC_frame.data[12] + NC_frame.data[13];
                int ip_dst = 256*256*256*NC_frame.data[14] + 256*256*NC_frame.data[15] + 256*NC_frame.data[16] + NC_frame.data[17];
                //ap_handle.ack_Frame(ip_src, seq_num_ack);

                std::vector<uint8_t> data_vector(NC_frame.data + 18, NC_frame.data + 18 + 11);
                ap_handle.received_Frame(NC_frame.seq_num, ip_dst, ip_src, data_vector);


            }
            else
            {
                int ip_src = 256*256*256*NC_frame.data[8] + 256*256*NC_frame.data[9] + 256*NC_frame.data[10] + NC_frame.data[11];
                int ip_dst = 256*256*256*NC_frame.data[12] + 256*256*NC_frame.data[13] + 256*NC_frame.data[14] + NC_frame.data[15];
                std::vector<uint8_t> data_vector(NC_frame.data + 16, NC_frame.data + 16 + 11);
                ap_handle.received_Frame(NC_frame.seq_num, ip_dst, ip_src, data_vector);

                //printf("      receive data %s\n", NC_frame.data); 
            }
        }



        //data[0] = total_frame%256;
        uint32_t delay = (rand()%30)*FRAME_IDLE_UNIT;
        usleep(delay);        
  //    std::cout << std::endl << "            [Host] START new frame" << std::endl;
        //std::cout<<"checking for next frame\n";
        if(ap_handle.AP_Has_Next() > 0 && (count % 2) == 0)
        {
            //std::cout << "getting next frame\n";
            sending_frame = ap_handle.get_Next();
            //std::cout << "got frame\n";
            if(use_NC && ap_handle.has_Matching_Frame(sending_frame))
            {
                std::cout << "\n\n\n\n NC!!!!!!!!!!!!!!!!!!!!! \n\n\n\n";
                IP_NC_Frame frame_to_code = ap_handle.get_NC_Frame();

                //set NC header and LLC header
                data_start = LLC_Header_Size + NC_L25_Header_Size + L3_Header_Size;
                L3_start = data_start - L3_Header_Size;

                // LLC Header for layer 2.5
                data[0] = 0xAA;
                data[1] = 0XAA;
                data[2] = 0X03;
                data[3] = 0XAA;
                data[4] = 0XAA;
                data[5] = 0XAA;
                data[6] = 0XAA;
                data[7] = 0XAA;
                
                int seq_num1 = sending_frame.seq_num_from;
                int seq_num2 = frame_to_code.seq_num_from;
                int ip_addr1 = sending_frame.ip_dst;
                int ip_addr2 = frame_to_code.ip_dst;

                // NC Header for data
                data[8] = seq_num1/256;
                data[9] = seq_num1;
                data[10] = seq_num2/256;
                data[11] = seq_num2;
                data[12] = 0;
                data[13] = 0;
                data[14] = 0;
                data[15] = ip_addr1;
                data[16] = 0;
                data[17] = 0;
                data[18] = 0;
                data[19] = ip_addr2;

                data[L3_start + 0] = 0;
                data[L3_start + 0] = 0;
                data[L3_start + 0] = 0;
                data[L3_start + 0] = sending_frame.ip_src^frame_to_code.ip_src;

                data[L3_start + 0] = 0;
                data[L3_start + 0] = 0;
                data[L3_start + 0] = 0;
                data[L3_start + 0] = sending_frame.ip_dst^frame_to_code.ip_dst;

                for( int i = 0; i < 11; i ++)
                {
                    data[data_start + i] = sending_frame.data[i]^frame_to_code.data[i];
                } 

                toAddr[0] = 0xFF;
                toAddr[1] = 0xFF;
                toAddr[2] = 0xFF;
                toAddr[3] = 0xFF;
                toAddr[4] = 0xFF;
                toAddr[5] = 0xFF;
                //ap_handle.set_NC_Ack(sending_frame.seq_num_from, sending_frame.ip_dst);
                //ap_handle.set_NC_Ack(frame_to_code.seq_num_from, frame_to_code.ip_dst);
                std::cout << "coded frames so far: " << frames_coded ++ << "out of total" << frames_coded + frames_uncoded;

            }
            else
            {

                //set LLC header to standard val
                data_start = LLC_Header_Size + L3_Header_Size;
                L3_start = data_start - L3_Header_Size;
                //std::cout<< "making header\n";
                //LLC Header for normal
                data[0] = 0xAA;
                data[1] = 0XAA;
                data[2] = 0X03;
                data[3] = 0X00;
                data[4] = 0X00;
                data[5] = 0X00;
                data[6] = 0X08;
                data[7] = 0X00;


                // source ip addr
                data[L3_start + 0] = 0;
                data[L3_start + 1] = 0;
                data[L3_start + 2] = 0;
                data[L3_start + 3] = sending_frame.ip_src;

                // dest ip addr
                data[L3_start + 4] = 0;
                data[L3_start + 5] = 0;
                data[L3_start + 6] = 0;
                data[L3_start + 7] = sending_frame.ip_dst;

                //std::cout << sending_frame.data.size() << " getting data\n";
                for( unsigned int i = 0; i < sending_frame.data.size(); i ++)
                {
                    std::cout << sending_frame.data[i];
                    data[data_start + i] = sending_frame.data[i];
                }
                //std::cout <<  "getting addr\n";
                std::vector<uint8_t> to_mac = IP_to_MAC_map[sending_frame.ip_dst];
                //std::cout << "setting addr\n";
                toAddr[0] = to_mac[0];
                toAddr[1] = to_mac[1];
                toAddr[2] = to_mac[2];
                toAddr[3] = to_mac[3];
                toAddr[4] = to_mac[4];
                toAddr[5] = to_mac[5];
                //std::cout<< "going to " << (int)toAddr[4] << (int)toAddr[5] << "\n";
                std::cout << "uncoded frames so far: " << frames_uncoded++ << "out of total" << frames_coded + frames_uncoded;


            }
            //std::cout << "about to send\n";
            status = handler.send_frame(data, toAddr, apAddr, sizeof(data));
            //std::cout << "just sent\n";
            
            total_frame++;



            switch(status){
            case 0:
                success_frame++;
                std::cout << std::endl << "        [Host] Success! " << success_frame << "/" << total_frame << std::endl;
                ap_handle.pop_Next_Frame();
                break;
            case -1:  
                std::cout << std::endl << "            [Host] Failure. discard." << std::endl;
                break;
            case -2:
                std::cout << std::endl << "            [Host] Failure! (busy)" << std::endl;
                break;   
            }     
        }
     
    }
    
}