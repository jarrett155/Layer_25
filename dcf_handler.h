//Written by Benjamin Boyle at University of Washington

#ifndef DCF_HANDLER_H
#define	DCF_HANDLER_H


#include "stdint.h"
#include "unistd.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pthread.h"

#include "phy_handler.h"

#include <queue>
#include <map>

#define MPDU 2048;
#define TIMEOUT 2560098

#define CRC_LENGTH 4

// frame control bits and masks
#define DATA_FRAME 0x0008
#define CTRL_FRAME 0x0004
#define SUBTYPE_ACK 0x000D
#define SUBTYPE_DATA 0x0000
#define SUBTYPE_RTS 0x000B
#define SUBTYPE_CTS 0x000C
#define PROTOCOL_AND_TYPE_MASK 0x000F
#define SUBTYPE_MASK 0X00F0
#define FOUR_BIT_MASK 0x0F
#define RETRY_BIT_MASK 0x0800
#define MORE_FRAG_MASK 0x0400
#define TYPE_SUBTYPE_MASK 0x00FF

#define MAC_ADDR_LENGTH 6

// clocks time conversion
#define NANO_PER_SEC 1000000000
#define MICRO_PER_SEC 1000000
#define NANO_PER_MICRO 1000




//time parameter to configure NAV
#define ACK_PHY_TIME 360              //obtained from matlab, either RTT_estimation or Detection_estimation
#define CTS_PHY_TIME 360               //same as ACK_PHY_TIME
#define MAC_RX_PROC_TIME 10             //can be measured by enabling DCF_MEASURE_MAC_PROC_ENABLED inside dcf_log.h, it is very small
#define ACK_DETECT_TO_RECEIVE_TIME 230  //obtained from matlab Detection_estimation

#define SLEEP_STEP_US 50

#define SEND_PHY_BUSY 171    //RANDON

//for code reading convenience 
#define RETURN_WITHOUT_TAKING_CHANNEL -1
#define RETURN_WITH_TAKING_CHANNEL    -1
#define RETURN_UNCLEAR_IF_TAKING_CHANNEL -1


// frame size
#define ACK_FRAME_SIZE 14
#define CTS_FRAME_SIZE 14
#define RTS_FRAME_SIZE 20

//used for probing 
#define PROBE_SLOT_NUM 2

//RXed frame 0->DATA 1->ACK/CTS 2-> RTS
#define DEDUCE_TO_BE_DATA 0
#define DEDUCE_TO_BE_ACK_OR_CTS 1
#define DEDUCE_TO_BE_RTS 2

struct DCFsettings{
    unsigned int frag_limit;
    uint32_t rts_limit;
    
    unsigned int aSlotTime;
    uint16_t aSIFSTime;
    uint16_t aDIFSTime;
    
    uint8_t MAC_address[6];
    uint8_t BSSID[6];
    uint8_t broadcast_adx[6];
    
    unsigned int CWmin;
    unsigned int CWmax;
    unsigned int short_retry_limit;
    unsigned int long_retry_limit;
    
    unsigned int ack_timeout;
    
    bool PwrMngt;
    
    bool is_FPGA_callback_used;
    unsigned short MCS_index;
    
};

struct DCFstatus{
    bool have_queued_frame;
    bool have_received_frame;
    bool have_cleared_to_send;
    bool have_ack;
    bool need_ack;       //after timeout, any intended frame is disregarded
    
    int slot;
    int success;
    unsigned int CW;
    uint16_t aNAVTimer;
    unsigned int aIFSTimer;

    unsigned int short_retries;
    unsigned int long_retries;
    
    bool is_first_frame_and_first_tx;  //for DIFS Waiting
    bool is_DIFS_preempted;
    bool is_RTS;
    bool has_CTS;
    bool need_CTS;       //after timeout, any intended frame is disregarded
    bool tx_not_preempt;
    
    bool frame_ctrl_retry;
    
    bool detect_preamble;
    int received_frame;            // 0->DATA 1->ACK/CTS 2-> RTS
    
    bool tx_threading_sending;
    bool detect_preamble_inside_probe;
    bool is_probing;
    
    uint8_t rx_phy_header[16];
};

/** frame formats*/
enum Frame_ctrl
{
    toDs = 0, fromDs, moreFrags, retry, pwrMngt, moreData, wep, order
};

struct frame_header_80211_DATA{
    uint16_t frame_ctrl;
    uint16_t dur_id;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
    uint16_t seq_ctl;
   // uint8_t addr4[6];
};

struct frame_80211_RTS
{       
    uint16_t frame_ctrl;
    uint16_t dur_id;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t fcs[4];  
};

struct frame_80211_ACK_or_CTS{
    uint16_t frame_ctrl;
    uint16_t dur_id;
    uint8_t addr1[6];
    uint8_t fcs[4];        
};

struct frame_80211{
    struct frame_header_80211_DATA header;
    uint8_t buffer_and_fcs[4000] ;
};

/** used in rate adaptation*/
struct Rate_record
{
    unsigned int consec_success;
    unsigned int consec_failure;
    unsigned int curr_threshold;
    bool is_first;
    bool is_first_retried;
    unsigned short current_rate;        
};

/** the structure of the shared queue between DCF and MAC*/
struct Queue_frame
{
    struct frame_80211 frame;
    uint32_t framesize;     //entire frame length
};

/** deliver data to MAC layer*/
struct RX_mac_frame
{
    struct frame_80211 * frame;
    uint32_t framesize;    
};

        
class Dcf_handler{
public:    
    Dcf_handler();
    Dcf_handler(struct DCFsettings);
    
    int start_dcf();
    int stop_dcf();
    int send_frame(uint8_t frame[],uint16_t framesize, std::queue<Queue_frame> * queue); //for DCF test #6 and #7
    int send_frame(std::queue<Queue_frame> * queue);    //for MAC call without rate adaptation
    int send_frame(std::queue<Queue_frame> * queue, Rate_record* record);    //for MAC call without rate adaptation         
    
    int retreive_frame(); 
    
    int set_myadx(uint8_t adx[6]);
    void set_PHY_TX_MCS(int MCS);    
    
    //set up callback function when a frame is received
    typedef void(*DCF_CallbackFunctionPtr)(void*, RX_mac_frame*);
    void dcf_setRxCallback_context(DCF_CallbackFunctionPtr, void * context);
    DCF_CallbackFunctionPtr m_dcf_callback_context;  //the function it is going to call
    void * m_dcf_context; // the context it is going to call
        
    friend class Mac_handler;
    
private:    
    int init_threads();
    int destroy_threads();
    int shutdown_threads();
    void * DCF_TX_thread();        
    int tx_frame();        
    
    uint16_t compute_RTS_DATA_duration (uint16_t frame_control, uint32_t next_tx_length, unsigned short tx_data_rate);
    
    void set_retry();
    int set_nav(uint16_t new_NAV, bool is_increment);
    void use_FPGA_callback_in_DCF(bool is_use_FPGA);    
    
    void create_RTS_frame (Queue_frame & next_tx_frame, unsigned short MCS_index);
    void create_CTS_frame (frame_80211_RTS * );
    void create_ACK_frame (frame_80211 * );
    
    pthread_mutex_t dcf_TX_RX_mutex = PTHREAD_MUTEX_INITIALIZER;
    int send_phy_packet(tx_data *);
    
    phy_handler handle;
    
    static void * tx_thread_helper(void * context)
    {
        return ((Dcf_handler * )context)->DCF_TX_thread();
    }
    
    //callback ACK RTS CTS done at host
    static void rx_callback_helper(void * context, rx_data * packet_rx )
    {
        ((Dcf_handler *)context)->DCF_RX_callback(packet_rx);
    }
    void DCF_RX_callback( rx_data * packet_rx );
    
    //callback set nav from FPGA
    static void rx_callback_FPGA_set_nav_helper(void * context, uint16_t rx_nav )
    {
        ((Dcf_handler *)context)->DCF_RX_callback_FPGA_set_nav(rx_nav);
    }        
    void DCF_RX_callback_FPGA_set_nav(uint16_t rx_nav);
    
    //callback for reading data from FPGA
    static void rx_callback_FPGA_data_helper(void * context, rx_data * data_packet )
    {
        ((Dcf_handler *)context)->DCF_RX_callback(data_packet);
    }
    void DCF_RX_callback_FPGA_data(rx_data * data_packet);
    
    //set up preamble callback, 
    static void callback_find_phy_preamble_helper(void * context )
    {
        ((Dcf_handler *)context)->DCF_callback_find_phy_preamble();
    }
    void DCF_callback_find_phy_preamble();
    
    static void callback_find_phy_header_helper(void* context, int mcs, int length, bool crc_valid )
    {
        ((Dcf_handler *)context)->DCF_callback_find_phy_header(mcs, length, crc_valid);
    }
    void DCF_callback_find_phy_header(int mcs, int length, bool crc_valid);
    
    // Status and Settings
    struct DCFsettings dcf_settings;
    struct DCFstatus dcf_status;
    
    // PHY packets
    struct rx_data m_rx_packet;

    struct tx_data m_tx_packet;
    struct tx_data m_txack_packet;  
    struct tx_data m_tx_rts_packet;
    
    // RTS frames
    struct frame_80211_RTS m_tx_RTS;
    struct frame_80211_RTS m_rx_RTS;
    
    // ACK or CTS frames
    struct frame_80211_ACK_or_CTS m_rx_ack;
    struct frame_80211_ACK_or_CTS m_tx_ack;       
    
    uint16_t total_frame_size;
    
    std::queue <Queue_frame> * queue_dcf;
    
    //tx thread
    pthread_t tx_thread;
    
    pthread_cond_t DCF_ready = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t DCF_lock = PTHREAD_MUTEX_INITIALIZER;
    
    pthread_cond_t DCF_tx_request = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t DCF_tx_request_lock = PTHREAD_MUTEX_INITIALIZER;        
        
    struct timespec SLOT_timer, rx_SIFS_timer, tx_SIFS_timer, sleepstep, probe_time;
    
    pthread_mutex_t read_abs_curr_mutex = PTHREAD_MUTEX_INITIALIZER;
    
    struct timespec nav_abs_time;
    struct timespec curr_abs_time;
    struct timespec difs_abs_time;
    struct timespec added_nav_time;  
    struct timespec rx_probe_curr_time;  
    struct timespec rx_probe_end_time;  
    
    bool is_abs_nav_greater_abs_curr();
    void set_abs_nav(uint16_t new_NAV);    
    bool is_abs_difs_greater_abs_curr();
    void set_abs_difs();    
    void timespec_add_us(struct timespec *t, long us);
    bool is_t1_greater_or_equal_t2(struct timespec *t1, struct timespec *t2);
    void timespec_t1_add_to_t2(struct timespec *t1, long us, struct timespec *t2);
    bool is_abs_nav_greater_abs_curr_result;
    bool is_abs_difs_greater_abs_curr_result;   
        
    bool is_rate_adapt;
    struct Rate_record* curr_record;
    
    int dcf_total_send_times,tx_success, pre_success,pre_total_send; //evaluate DCF performance
    
 };

#endif	/* DCF_HANDLER_H */