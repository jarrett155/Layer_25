#ifndef MAC_HANDLER_H
#define	MAC_HANDLER_H
#include "dcf_handler.h"
#include <queue>

#define SEQUENCE_NUM_MOD 4096

// rate adaptation
#define MAX_RATE_MODE 7
#define MIN_RATE_MODE 0
#define FRAME_IDLE_UNIT 0

/** deliver data to Network layer*/
struct RX_datagram
{
    uint8_t * data;
    uint32_t data_size;
    uint8_t * source_addr;
    uint16_t seq_num;
};

struct Rate_adapt_para{
    unsigned int threshold_constant;        //rate adapt   consecutive success
    unsigned int threshold_multiple;        //rate adapt binary back off
    unsigned int failure_threshold;         //rate adapt greatest acceptable consecutive failure    
};

struct Mac_setting
{
    uint8_t my_mac_addr[6];
    uint16_t frag_limit;   
    bool rate_adapt_switch;
    unsigned short MCS_index;
    bool is_FPGA_callback_used;
    bool is_AP;
    bool queue_frames;
    std::queue<RX_datagram> *frame_queue;
};

struct Mac_status
{
    uint16_t seq_number;                       //used at transmitter side
    uint16_t curr_frag_number;                 //used at transmitter side
    std::map<uint64_t, uint16_t> seq_ctrl_map;  //for error control duplicate packet, used at receiver side
    
    bool have_queued_frame;
    int transmission_success;
    
    std::map<uint64_t,Rate_record> rate_map;  //rate adaptation
    Rate_record *curr_record;
};


class Mac_handler
{
public :
    Mac_handler();
    int set_myadx(uint8_t adx[6]);
    void enable_rate_adaptation();
    void disable_rate_adaptation();
    int start_mac();
    int send_frame(uint8_t* payload, uint8_t* des_addr, uint16_t length);
    void use_FPGA_callback_in_DCF(bool is_use_FPGA);
    void set_PHY_TX_MCS(unsigned short MCS);
    void set_AP(bool is_set_to_AP);
    int send_frame(uint8_t* payload, uint8_t* des_addr, uint8_t* AP_addr, uint16_t length);
    void set_AP_addr(uint8_t adx[6]);
    int get_seq_num(); // Get the seq number of the most recently sent frame
    void set_frame_queue(std::queue<RX_datagram> *queue_for_frames);

    //set up callback function when a frame is received, it will be used by main.cpp
    typedef void(*MAC_CallbackFunctionPtr)(void*, RX_datagram*);
    void mac_setRxCallback_context(MAC_CallbackFunctionPtr, void * context);
    MAC_CallbackFunctionPtr m_upper_mac_callback_helper;  //the function is going to be called
    void * m_upper_mac_context; // the context it is going to be called in main   
    void mac_setRxCallback(void (*rx_callback) (struct RX_datagram* data));
    friend class Dcf_handler; //so that dcf can access queue
private:
    void * MAC_TX_thread();     
   
    uint32_t create_data_frame (uint16_t max_frag_number);    
    
    void decide_record();
    void rate_adaptation();

    void queue_frame(frame_80211* frame, uint32_t length);
    
    //used for transmitting thread
    static void * mac_tx_thread_helper(void * context)
    {
        return ((Mac_handler *)context)->MAC_TX_thread();
    }
    
    //used for setting up callback inside DCF 
    static void mac_rx_callback_helper(void* context, RX_mac_frame * frame)
    {
        ((Mac_handler *)context) -> mac_RX_callback(frame);
    }      
    void mac_RX_callback(RX_mac_frame * );
    
    static void mac_AP_callback_helper(void* context, RX_mac_frame * frame)
    {
        ((Mac_handler *)context) -> mac_AP_callback(frame);
    }      
    void mac_AP_callback(RX_mac_frame * );
    uint16_t create_from_AP_frame(uint8_t * data, uint16_t length, uint8_t *DA, uint8_t *SA);
    uint16_t create_to_AP_frame(uint8_t * data, uint16_t length, uint8_t *DA, uint8_t *AP);
    void push_relay_frame(frame_80211 *frame, uint16_t length);
    
    struct frame_80211 relay_frame;
    
    struct Mac_setting mac_settings;
    struct Mac_status mac_status;
    struct frame_80211 m_tx_frame;                      
    
    uint8_t* destination_mac_addr;
    uint8_t* datagram;
    uint8_t* ap_mac_addr;
    uint16_t data_length; 
        
    RX_datagram deliver_data;
    uint8_t receiver_buffer[8000];    //decide a better value, 
    uint32_t receiver_buffer_length;
    
    void (*mac_rx_call_up) (struct RX_datagram* mac_data);
    
    pthread_t mac_tx_thread;
    
    pthread_cond_t MAC_ready = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t MAC_lock = PTHREAD_MUTEX_INITIALIZER;
    
    pthread_cond_t MAC_tx_request = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t MAC_tx_request_lock = PTHREAD_MUTEX_INITIALIZER;
    
    pthread_mutex_t MAC_queue = PTHREAD_MUTEX_INITIALIZER;
    
    Dcf_handler dcf_handler;
    
    Rate_adapt_para rate_adapt_para;
    
    std::queue<Queue_frame> queue;
};

#endif	/* MAC_HANDLER_H */