//Written by Benjamin Boyle at University of Washington

#include "dcf_handler.h"
#include "dcf_log.h"
#include <stdarg.h>
#include <sys/time.h>
#include <errno.h>

/** control accurate waiting time in software */
struct timespec tx_nano_start, tx_nano_stamp;
struct timespec rx_nano_start, rx_nano_stamp;

/** timer variables to record the execution time for a period*/
struct timeval tx_micro_start,tx_micro_end, probe_start, probe_end;
struct timeval rx_micro_start,rx_micro_end, rx_DCF_start,rx_DCF_end;
unsigned int rx_timer = 0;
unsigned int tx_timer = 0;

/** timestamp */
struct timeval dcf_start_time, dcf_now;
double time_elapsed = 0;

char dcf_logname [50]= "default_dcf.log";

/** Constructor */
Dcf_handler::Dcf_handler()
{
            // addr 1 is RA / DA address
            // addr2 is TA/ BSSID
            // add3 is the source address

    memset(dcf_settings.broadcast_adx,0xFF,6);
    memset(dcf_settings.MAC_address,0x00,6);
    dcf_settings.PwrMngt = 0;
    dcf_settings.frag_limit = 2030;                          //if length of entire frame (including headers)higher than this value, frag is used,
    dcf_settings.rts_limit = 2030;        //if length of entire frame higher than or equal to this value, rts is sent
                                                             //for datagram size of 2000, frag_limit = 2028 doesn't fragment but use RTS,
                                                             //size of 2029 doesn't use fragment or RTS, size of 2027 use fragment and RTS
    dcf_settings.aSlotTime =200;  //obtained from Matlab, 580 can be less to trade off high probability of timing mismatch (collision), but it can reduce idle time
    dcf_settings.aSIFSTime = 50;  //not used, assume the MAC process while running code is SIFS
    dcf_settings.aDIFSTime = ( 2 * dcf_settings.aSlotTime + MAC_RX_PROC_TIME);
    dcf_settings.MAC_address[0] = 0xAA;
    dcf_settings.BSSID[0] = 0xCC;
    dcf_settings.CWmin = 5; // Given by standard, 2^5-1 = 31
    dcf_settings.CWmax = 10; // Given by standard, 2^10-1 = 1023
    dcf_settings.short_retry_limit = 5; // find good value
    dcf_settings.long_retry_limit = 5; //find a good value

    dcf_settings.is_FPGA_callback_used = false;
    dcf_settings.MCS_index = 0;

    dcf_status.frame_ctrl_retry = false;
    dcf_settings.ack_timeout = 20000;// 20 ms

    m_txack_packet.mode = BPSK_1By2_1p8Mbps;

    sleepstep.tv_sec = 0;
    sleepstep.tv_nsec = SLEEP_STEP_US * NANO_PER_MICRO;

    is_rate_adapt = false;

    dcf_status.have_queued_frame = 0;
    dcf_status.have_ack = 0;
    dcf_status.success = -1;
    dcf_status.short_retries = 0;
    dcf_status.long_retries = 0;
    dcf_status.detect_preamble = false;
    dcf_status.tx_not_preempt = true;
    probe_time.tv_nsec = dcf_settings.aSlotTime*PROBE_SLOT_NUM*NANO_PER_MICRO;
    probe_time.tv_sec = 0;

    is_abs_nav_greater_abs_curr_result = false;
    is_abs_difs_greater_abs_curr_result = false;

    dcf_status.is_probing = false;
    dcf_status.detect_preamble_inside_probe = false;
}

Dcf_handler::Dcf_handler(struct DCFsettings settings):
dcf_settings(settings)
{}

/**
 * Open the DCF threads and phy handler
 * Setup the log file for each board, name is in the form DCF_Board_[mac_addr].log
 */
int Dcf_handler::start_dcf(){
    int status;
    char filename[50];
    uint8_t * addr = dcf_settings.MAC_address;
    sprintf(filename,"DCF_Board_%02x%02x%02x%02x%02x%02x.log",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
    strcpy(dcf_logname,filename);
    m_tx_packet.mode = IEEE80211TVWSModes(dcf_settings.MCS_index);

    #ifdef DCF_TX_CONTENT_LOGGING_ENABLED
    tx_content_log_info(dcf_settings.MAC_address,"%-10s%-10s%-10s%-10s%-10s%-10s%-14s%-14s%-18s%-18s\n",
                                                "Tx result","Mode","cons_suc","cons_fai","RTT us","send time","myAddr","toAddr","frame ctrl","seq_ctrl");
    #endif
    gettimeofday(&dcf_start_time,NULL);

    dcf_total_send_times = 0;
    tx_success = 0;
    pre_success = 0;
    pre_total_send = 0;
    clock_gettime(CLOCK_MONOTONIC, &nav_abs_time);
    clock_gettime(CLOCK_MONOTONIC, &curr_abs_time);

    if(dcf_settings.is_FPGA_callback_used)
        handle.setRxCallback_context( Dcf_handler::rx_callback_FPGA_data_helper, this);
    else
        handle.setRxCallback_context( Dcf_handler::rx_callback_helper, this);
    handle.setRxSyncCallback_context(&(Dcf_handler::callback_find_phy_preamble_helper), this);
    handle.setRxHdrCallback_context(&(Dcf_handler::callback_find_phy_header_helper),this);

    handle.start_phy(1);
    status = pthread_create(&tx_thread, NULL, this->tx_thread_helper, this);
    dcf_log_info((char*)&dcf_logname,"my MAC address is  0x%02x:%02x:%02x:%02x:%02x:%02x. \n",
            addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);

    if (!status)
        dcf_log_info((char*)&dcf_logname,"tx threads initialized.\n");
    else
        dcf_log_error("DCF thread initialization Fails\n");

    return status;
}

int Dcf_handler::stop_dcf(){
    //Stop the DCF
    //addthread kill
    pthread_cancel(tx_thread);
    handle.stop_phy();

    return -1;
}

/**
 * Add a frame the the DCF and compute fcs, it will be changed later if MAC is added
 * It returns three possible states
 * 0 for Success; -1: transmission Failure ;-2 for previous frame unsent Failure
 * potential cause for -2 is when two threads are in the caller functions
 */
int Dcf_handler::send_frame(uint8_t frame[], uint16_t framesize, std::queue<Queue_frame> * queue){
    is_rate_adapt = false;
    if (!dcf_status.have_queued_frame)
    {
        queue_dcf = queue;
        Queue_frame frame_txing;
        frame_80211 m_tx_frame;
        memcpy(&m_tx_frame,frame,framesize);
        frame_txing.frame = m_tx_frame;
        frame_txing.framesize = framesize;
        queue_dcf->push(frame_txing);
        dcf_status.is_first_frame_and_first_tx = true;
        dcf_status.has_CTS = false;
        dcf_status.is_RTS = false;

        if(is_abs_nav_greater_abs_curr())
        {
            int window = pow(2,dcf_settings.CWmin)-1;
            dcf_status.slot = rand()% window;
            dcf_status.is_first_frame_and_first_tx = false;
        }
        dcf_status.have_queued_frame = 1;
        pthread_cond_broadcast(&DCF_tx_request);
    }
    else
    {
        dcf_log_error("previous frame still unsent\n");
        dcf_status.success =  -2;
    }
    dcf_log_info((char*)&dcf_logname,"DCF is working on it\n");
    pthread_mutex_lock(&DCF_lock);
    while((dcf_status.have_queued_frame))      //if dcf have a frame to send
        pthread_cond_wait(&DCF_ready, &DCF_lock);
    pthread_mutex_unlock(&DCF_lock);
#ifdef DISPLAY_DCF_TX_STATISTICS
    if(dcf_total_send_times-pre_total_send >= 100)
    {
        printf("#%d TX success percent %.2f%% in 100 tx, accum %.2f%%\n\n", dcf_total_send_times, 100*(tx_success-pre_success)/(double)(dcf_total_send_times-pre_total_send),
                (double)(100*tx_success)/dcf_total_send_times);
        pre_success =  tx_success;
        pre_total_send = dcf_total_send_times;
    }
#endif
    dcf_log_info((char*)&dcf_logname,"DCF is ready for new frame\n");
    return dcf_status.success;
}

int Dcf_handler::send_frame(std::queue<Queue_frame> * queue)
{
    is_rate_adapt = false;
    if (!dcf_status.have_queued_frame)
    {
        queue_dcf = queue;
        dcf_status.is_first_frame_and_first_tx = true;
        dcf_status.has_CTS = false;
        dcf_status.is_RTS = false;

        if(is_abs_nav_greater_abs_curr())
        {
                int window = pow(2,dcf_settings.CWmin)-1;
                dcf_status.slot = rand()% window;
                dcf_status.is_first_frame_and_first_tx = false;
        }
        dcf_status.have_queued_frame = 1;
        pthread_cond_broadcast(&DCF_tx_request);
    }
    else
    {
        dcf_log_error("previous frame still unsent\n");
        dcf_status.success =  -2;
    }
    dcf_log_info((char*)&dcf_logname,"DCF is working on the new frame\n");

    pthread_mutex_lock(&DCF_lock);
    while((dcf_status.have_queued_frame))      //if dcf have a frame to send
        pthread_cond_wait(&DCF_ready, &DCF_lock);
    pthread_mutex_unlock(&DCF_lock);
#ifdef DISPLAY_DCF_TX_STATISTICS
    if(dcf_total_send_times-pre_total_send >= 100)
    {
        printf("#%d TX success percent %.2f%% in 100 frm, accum %.2f%%\n\n", dcf_total_send_times, 100*(tx_success-pre_success)/(double)(dcf_total_send_times-pre_total_send),
                (double)(100*tx_success)/dcf_total_send_times);
        pre_success =  tx_success;
        pre_total_send = dcf_total_send_times;
    }
#endif
    dcf_log_info((char*)&dcf_logname,"DCF is ready for new frame\n");
    return dcf_status.success;
}

/** send frame function for generic MAC test with rate adaptation*/
int Dcf_handler::send_frame(std::queue<Queue_frame> * queue, Rate_record* record)    //for MAC call
{
    curr_record = record;
    is_rate_adapt = true;
    if (!dcf_status.have_queued_frame)
    {
        queue_dcf = queue;
        dcf_status.is_first_frame_and_first_tx = true;
        dcf_status.has_CTS = false;
        dcf_status.is_RTS = false;

        if(is_abs_nav_greater_abs_curr())
        {
                int window = pow(2,dcf_settings.CWmin)-1;
                dcf_status.slot = rand()% window;
                dcf_status.is_first_frame_and_first_tx = false;
        }
        dcf_status.have_queued_frame = 1;
        pthread_cond_broadcast(&DCF_tx_request);
    }
    else
    {
        dcf_log_error("previous frame still unsent\n");
        dcf_status.success =  -2;
    }
    dcf_log_info((char*)&dcf_logname,"DCF is working on it\n");
    pthread_mutex_lock(&DCF_lock);
    while((dcf_status.have_queued_frame))      //if dcf have a frame to send
        pthread_cond_wait(&DCF_ready, &DCF_lock);
    pthread_mutex_unlock(&DCF_lock);
#ifdef DISPLAY_DCF_TX_STATISTICS
    if(dcf_total_send_times-pre_total_send >= 100)
    {
        printf("#%d TX success percent %.2f%% in 100 frm, accum %.2f%%\n\n", dcf_total_send_times, 100*(tx_success-pre_success)/(double)(dcf_total_send_times-pre_total_send),
                (double)(100*tx_success)/dcf_total_send_times);
        pre_success =  tx_success;
        pre_total_send = dcf_total_send_times;
    }
#endif
    dcf_log_info((char*)&dcf_logname,"DCF is ready for new frame\n");
    return dcf_status.success;
}



int Dcf_handler::retreive_frame(){
    if(1 == dcf_status.have_queued_frame){
        std::cout << std::endl<< "DCF: ERROR No frame received to retreive" << std::endl;
    }else{
        std::cout << std::endl<< "Retreived frame... " << std::endl;
        dcf_status.have_queued_frame = 0;

    }
    // Get a frame from the DCF
    // Eventually add a buffer?
    // Definitely will need a buffer some day
    // linked list?
    // Not sure, need to figure it out
    dcf_status.have_received_frame = 0;
    std::cout << std::endl<< "DCF: Retreiving a frame from the top of the DCF buffer" << std::endl;
    std::cout << std::endl<< m_rx_packet.buffer << std::endl;
    return -1;
}

/** this functions set the address of the board*/
int Dcf_handler::set_myadx(uint8_t adx[6]){
    memcpy(&dcf_settings.MAC_address,adx,6);
    return -1;
}

/**
 * It does the actual work of transmitting a frame. Before transmitting, DCF
 * should wait for DIFS to check if channel is idle. The NAV is the only source
 * for this checking procedure.
 * @return
 */
int Dcf_handler::tx_frame(){
    int status;
    dcf_status.have_ack = 0;

    Queue_frame & frame_txing = queue_dcf->front();
    if (frame_txing.framesize >= dcf_settings.rts_limit)
        dcf_status.need_CTS = true;
    else
        dcf_status.need_CTS = false;

    if (dcf_status.need_CTS && !dcf_status.has_CTS) //check for RTS
    {
        dcf_status.is_RTS =  true;
        if(is_rate_adapt)
            create_RTS_frame(frame_txing, curr_record->current_rate);
        else
            create_RTS_frame(frame_txing, dcf_settings.MCS_index);
        m_tx_packet.mode = IEEE80211TVWSModes(0);
        memcpy(m_tx_packet.buffer,&m_tx_RTS,sizeof(m_tx_RTS));
        m_tx_packet.length = sizeof(m_tx_RTS);

        //Check for DIFS, only if it is the first frame in queue, also it has to be the first tx
        //Retry transmission goes through other path to transmit
        if (dcf_status.is_first_frame_and_first_tx)
        {
            //check if DIFS at first time is preempted,
            dcf_status.is_DIFS_preempted = false;
            set_abs_difs();
            do{
                clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepstep,NULL);
                if(dcf_status.detect_preamble || is_abs_nav_greater_abs_curr())
                {
                    dcf_log_info((char*)&dcf_logname," channel becomes busy, NAV is set by RX in DIFS \n");
                    dcf_status.is_DIFS_preempted = true;
                    return RETURN_WITHOUT_TAKING_CHANNEL;
                }
            } while(is_abs_difs_greater_abs_curr());
            dcf_log_info((char*)&dcf_logname,"Waited DIFS without channel busy, ready to TX\n");
        }

#ifdef DCF_TX_CONTENT_LOGGING_ENABLED
        gettimeofday (&tx_micro_start, NULL);         //for log measuring tx time
#endif
        status = send_phy_packet(&m_tx_packet);
        if (status != 0 )
        {
            dcf_log_info((char*)&dcf_logname,"TX send error\n");
            return RETURN_UNCLEAR_IF_TAKING_CHANNEL;
        }

#ifdef DCF_TX_CONTENT_LOGGING_ENABLED
        gettimeofday(&tx_micro_end,NULL);
#endif
        dcf_log_info((char*)&dcf_logname,"RTS SENT\n");
    }
    else // send non-fragmented or fragmented data
    {
        if (dcf_status.frame_ctrl_retry)
            set_retry();
        m_tx_packet.length = frame_txing.framesize;
        if (is_rate_adapt)
        {
            m_tx_packet.mode = IEEE80211TVWSModes(curr_record->current_rate);
        }
        memcpy(m_tx_packet.buffer,(uint8_t*)&(frame_txing.frame),frame_txing.framesize);

        if( frame_txing.frame.header.addr1[0] == 0XFF && frame_txing.frame.header.addr1[1] == 0XFF 
            && frame_txing.frame.header.addr1[2] == 0XFF && frame_txing.frame.header.addr1[3] == 0XFF 
            && frame_txing.frame.header.addr1[4] == 0XFF && frame_txing.frame.header.addr1[5] == 0XFF)
        {
            dcf_status.need_ack = false;
            dcf_status.have_ack = true;
        }
        else
        {
            dcf_status.need_ack = true;
            dcf_status.have_ack = false;
        }
        

        //Check for DIFS, only if it is the first frame in queue, also it has to be the first tx
        //Retry transmission goes through other path to transmit

        /*if (dcf_status.is_first_frame_and_first_tx)
        {
            //check if DIFS at first time is preempted,
            dcf_status.is_DIFS_preempted = false;
            set_abs_difs();
            do{
                clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepstep,NULL);
                if(dcf_status.detect_preamble || is_abs_nav_greater_abs_curr())
                {
                    //std::cout << "\033[1;33m" << "while NAV is waiting in DIFS, channel becomes busy by RX" << "\033[0m" << std::endl;
                    dcf_log_info((char*)&dcf_logname," channel becomes busy, NAV is set by RX in DIFS \n");
                    dcf_status.is_DIFS_preempted = true;
                    return RETURN_WITHOUT_TAKING_CHANNEL;
                }
            } while(is_abs_difs_greater_abs_curr());
            dcf_log_info((char*)&dcf_logname,"Waited DIFS without channel busy, ready to TX\n");
        }*/

#ifdef DCF_TX_CONTENT_LOGGING_ENABLED
        gettimeofday (&tx_micro_start, NULL);  //for collecting time for sending behavior
#endif
        status = send_phy_packet(&m_tx_packet);
        if (status != 0 )
        {
            dcf_log_info((char*)&dcf_logname,"TX send error\n");
            return RETURN_UNCLEAR_IF_TAKING_CHANNEL;
        }
        //set_abs_nav(ACK_PHY_TIME); 
#ifdef DCF_TX_CONTENT_LOGGING_ENABLED
        gettimeofday(&tx_micro_end,NULL);
#endif
        dcf_log_info((char*)&dcf_logname,"DATA SENT packet length %d\n",frame_txing.framesize);
    }
    dcf_total_send_times++;                    //for collecting DCF behavior statistics


    //Wait for ACK timeout or preemption if no response from RX callback, otherwise return
    unsigned long nTime;
    clock_gettime(CLOCK_MONOTONIC,&tx_nano_start);
    bool time_in_flag = true;

    while((!dcf_status.have_ack||(dcf_status.need_CTS&&!dcf_status.has_CTS)) && time_in_flag && dcf_status.tx_not_preempt)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepstep,NULL);
        clock_gettime(CLOCK_MONOTONIC,&tx_nano_stamp);
        nTime = (tx_nano_stamp.tv_sec - tx_nano_start.tv_sec)*NANO_PER_SEC + tx_nano_stamp.tv_nsec - tx_nano_start.tv_nsec;
        if (nTime >= dcf_settings.ack_timeout*NANO_PER_MICRO)
        {
            time_in_flag = false;
        }
    }

    /***********************log file start******************************/
#ifdef DCF_TX_CONTENT_LOGGING_ENABLED
        int mode = dcf_settings.MCS_index;
        int cons_success = 0;
        int cons_failure = 0;
        if (is_rate_adapt)
        {
            mode = curr_record->current_rate;
            cons_success = curr_record->consec_success;
            cons_failure = curr_record->consec_failure;
        }
        unsigned long send_time = (tx_micro_end.tv_sec - tx_micro_start.tv_sec) *MICRO_PER_SEC + tx_micro_end.tv_usec - tx_micro_start.tv_usec;
        char my_addr_string[20];
        char to_addr_string[20];
        bool tx_success;
        gettimeofday(&dcf_now,NULL);
        unsigned long RTT_DCF;
        if(dcf_status.have_ack || (dcf_status.need_CTS && dcf_status.has_CTS))
        {
            RTT_DCF = (dcf_now.tv_sec - tx_micro_start.tv_sec) *MICRO_PER_SEC + dcf_now.tv_usec - tx_micro_start.tv_usec;
            tx_success = true;
        }
        else
        {
            RTT_DCF = 999999;
            tx_success = false;
        }
        uint8_t * addr = m_tx_packet.buffer+4;  //locate to addr1
        sprintf(to_addr_string,"%02x%02x%02x%02x%02x%02x",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
        addr = m_tx_packet.buffer+10;           //locate to addr2
        sprintf(my_addr_string,"%02x%02x%02x%02x%02x%02x",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
        uint16_t frame_control_phy;
        memcpy(&frame_control_phy,m_tx_packet.buffer,2);
        tx_content_log_info(dcf_settings.MAC_address,"%-10d%-10d%-10d%-10d%-10ld%-10ld%-14s%-14s",tx_success,mode,cons_success,cons_failure,RTT_DCF,
                                                                                             send_time,my_addr_string,to_addr_string);
        tx_content_print_bit(dcf_settings.MAC_address,frame_control_phy);
        tx_content_log_info(dcf_settings.MAC_address,"  ");
        if (!dcf_status.is_RTS)
            tx_content_print_bit(dcf_settings.MAC_address,frame_txing.frame.header.seq_ctl);
        else
            tx_content_print_bit(dcf_settings.MAC_address,0xFFFF);
        tx_content_log_info(dcf_settings.MAC_address,"\n");
#endif
    /***********************log file end********************************/

    if(dcf_status.have_ack || (dcf_status.is_RTS && dcf_status.has_CTS))
    {
        dcf_log_info((char*)&dcf_logname,"ACK or CTS received   \n");
        return 0;
    }
    else
    {
        if (dcf_status.need_CTS)
            dcf_status.need_CTS = false;
        if (dcf_status.need_ack)
            dcf_status.need_ack = false;
        dcf_status.tx_not_preempt = true;
        dcf_log_info((char*)&dcf_logname,"NO RESPONSE TIMEOUT   \n");
        return -1;
    }
}



/**
 * It contains main structure of DCF. When a frame first time enter DCF, it check
 * if the channel is busy at entering moment. If the channel is not, then it
 * continue TX process, which is to check DIFS and transmit. For complete
 * description, please refer to plot written by Farzard.
 */
void  * Dcf_handler::DCF_TX_thread(){
    srand(time(NULL));
    int window = 0;
    dcf_status.CW = dcf_settings.CWmin;
    while(1){
        if((    dcf_status.have_queued_frame)
                &&(!is_abs_nav_greater_abs_curr())
                &&(0 >= dcf_status.slot)){
            dcf_log_info((char*)&dcf_logname,"both sLot, NAV at 0\n");
            if(0 == tx_frame()){  //Frame sent successfully.
                tx_success++; //to collect statistics of behavior of DCF
                dcf_status.frame_ctrl_retry = false;
                if(dcf_status.is_first_frame_and_first_tx){
                    dcf_status.is_first_frame_and_first_tx= false;
                }

                if(!dcf_status.is_RTS)     //exclude RTS from rate adaptation
                {
                    dcf_status.have_ack = 0;
                    dcf_log_info((char*)&dcf_logname,"\t\t\t SUCCESS A Fragment TX:%d/%d\n",tx_success,dcf_total_send_times);
                    queue_dcf->pop();
                }
                else
                {
                    dcf_status.is_RTS = false;
                    dcf_status.short_retries = 0;
                }

                if (queue_dcf->empty())
                {
                    dcf_status.success = 0;
                    dcf_status.short_retries = 0;
                    dcf_status.long_retries = 0;
                    dcf_status.CW = dcf_settings.CWmin;
                    dcf_status.have_queued_frame = 0;
                    dcf_log_info((char*)&dcf_logname,"\t\t\t SUCCESS A FRAME #%d TX:%d/%d\n",*(m_tx_packet.buffer+sizeof(frame_header_80211_DATA)) ,tx_success,dcf_total_send_times);
                    pthread_cond_broadcast(&DCF_ready);
                }
            }else if ((dcf_settings.short_retry_limit <= dcf_status.short_retries)||
                     ((dcf_settings.long_retry_limit <= dcf_status.long_retries))){
                //failure and report to upper layer
                while(!queue_dcf->empty())
                {
                    queue_dcf->pop();
                }

                dcf_status.frame_ctrl_retry = false;
                dcf_status.success = -1;
                dcf_status.short_retries = 0;
                dcf_status.long_retries = 0;
                dcf_status.CW = dcf_settings.CWmin;
                dcf_log_info((char*)&dcf_logname,"\t\t\t FAIL, OVERLIMIT, DISCARD\n");
                dcf_status.have_queued_frame = 0;
                pthread_cond_broadcast(&DCF_ready);
            }else{  //try again reroll slot, etc
                // if the first frame isn't preempted, i.e it possess channel
                if(!dcf_status.is_first_frame_and_first_tx || !dcf_status.is_DIFS_preempted){
                    dcf_status.short_retries++;
                    dcf_status.long_retries++;
                    dcf_status.frame_ctrl_retry = true;
                    if(dcf_status.CW < dcf_settings.CWmax){
                        dcf_status.CW = dcf_status.CW + 1;
                    }
                }
                if(dcf_status.is_first_frame_and_first_tx){
                    dcf_status.is_first_frame_and_first_tx= false;
                }
                window = pow(2,dcf_status.CW)-1;
                dcf_status.slot = rand()% window;
                //fail#0 is RTS, Data start from 1
                dcf_log_info((char*)&dcf_logname,"\t\t transmit %d th  FAILED, backoff %d slots\n", dcf_status.short_retries,dcf_status.slot);
            }

        }else if(is_abs_nav_greater_abs_curr()){   // channel busy NAV counts down
            do{
                clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepstep,NULL);
            } while(is_abs_nav_greater_abs_curr());

            //Wait DIFS every time after NAV counts to 0, DIFS is preemptible
            set_abs_difs();
            do{
                clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepstep,NULL);
                if(dcf_status.detect_preamble || is_abs_nav_greater_abs_curr())
                {
                    dcf_log_info((char*)&dcf_logname,"After NAV becomes 0 while in DIFS, channel becomes busy\n");
                    break;
                }
            } while(is_abs_difs_greater_abs_curr());
            dcf_log_info((char*)&dcf_logname,"Waiting DIFS: DONE, ready for %dslot--\n", dcf_status.slot);

        }else if(0 < dcf_status.slot){
            if(clock_gettime(CLOCK_MONOTONIC, &SLOT_timer)!=0)
                    perror("clock_gettime error");
            timespec_add_us(&SLOT_timer,dcf_settings.aSlotTime);
            if(clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&SLOT_timer,NULL) != 0)
                    printf("clock nanosleep error");
            dcf_status.slot--;
            dcf_log_info((char*)&dcf_logname,"slot to %d\n", dcf_status.slot);
        }else{
            pthread_mutex_lock(&DCF_tx_request_lock);
            pthread_cond_wait(&DCF_tx_request,&DCF_tx_request_lock);
            pthread_mutex_unlock(&DCF_tx_request_lock);
        }
    }
    pthread_exit(NULL);
}

/**
 * It processes data received from physical layer. If CRC is not good, everything
 * is discarded. The data delivery service to higher layer is not implemented yet.
 * @param rx_packet
 */

void Dcf_handler::DCF_RX_callback(rx_data * rx_packet)
{
    if((rx_packet->isFrameDetected)&&(rx_packet->CRC_valid)) //&& dcf_status.detect_preamble
    {
#ifdef DCF_MEASURE_MAC_PROC_ENABLED
        gettimeofday(&rx_DCF_start, NULL);
#endif
        dcf_status.detect_preamble = false;
        uint16_t protocol_and_type = 0x0000, subtype = 0x0000, frame_control = 0x0000;
        frame_control = rx_packet->buffer[0] | (uint16_t)rx_packet->buffer[1] << 8;
        protocol_and_type = frame_control & (uint16_t)PROTOCOL_AND_TYPE_MASK;
        subtype = (uint16_t)(((frame_control & (uint16_t)SUBTYPE_MASK)) >> 4) & (uint16_t)FOUR_BIT_MASK;
        
        dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tDCF processing data\n");
        if(((uint16_t)CTRL_FRAME ==  protocol_and_type) &&( (uint16_t)SUBTYPE_ACK == subtype ))  //receive an ACK
        {
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tReceive ACK length %d\n",rx_packet->length);
            frame_80211_ACK_or_CTS* m_rx_ack_ptr = (frame_80211_ACK_or_CTS*)rx_packet->buffer;
            if(0 == memcmp (m_rx_ack_ptr->addr1, dcf_settings.MAC_address,MAC_ADDR_LENGTH))     //to me
            {
                if (dcf_status.need_ack)
                    dcf_status.have_ack = 1;
            }
            else
            {
                set_abs_nav(m_rx_ack_ptr->dur_id);
            }
        }
        else if(((uint16_t)DATA_FRAME == protocol_and_type ) &&((uint16_t)SUBTYPE_DATA == subtype )) // receive a Data
        {
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tReceive DATA #%d length %d\n", *(rx_packet->buffer +sizeof(frame_header_80211_DATA)),rx_packet->length);
            frame_80211 * data_frame_rxing = (frame_80211 *)rx_packet->buffer;
            set_abs_nav(data_frame_rxing->header.dur_id);
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tNAV %d\n", data_frame_rxing->header.dur_id);
            if (rx_packet->length == sizeof(frame_80211_ACK_or_CTS))
                return;
            if(0 == memcmp(data_frame_rxing->header.addr1, dcf_settings.MAC_address, MAC_ADDR_LENGTH))                 // To me
            {
                //create ACK
                create_ACK_frame(data_frame_rxing);
                memcpy (m_txack_packet.buffer,&m_tx_ack, sizeof(frame_80211_ACK_or_CTS));
                m_txack_packet.length = sizeof(frame_80211_ACK_or_CTS);
#ifdef DCF_MEASURE_MAC_PROC_ENABLED
                gettimeofday(&rx_DCF_end, NULL);
                rx_timer = (rx_DCF_end.tv_sec-rx_DCF_start.tv_sec)*MICRO_PER_SEC + rx_DCF_end.tv_usec - rx_DCF_start.tv_usec;
                print_text_at_insert_location(dcf_settings.MAC_address,"%d\n", rx_timer);
#endif

#ifdef DCF_LOGGING_ENABLED
                int status = -1;
                status = send_phy_packet(&m_txack_packet);
#else
                send_phy_packet(&m_txack_packet);
#endif
                dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tsent ACK status %d\n", status);
                dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tACK sent with dur_id %d\n", m_tx_ack.dur_id);
                //create callback
                RX_mac_frame rx_frame;
                rx_frame.frame = data_frame_rxing;
                rx_frame.framesize = rx_packet->length;
                m_dcf_callback_context(m_dcf_context,&rx_frame);
            }
            else if(0 == memcmp(data_frame_rxing->header.addr1, dcf_settings.broadcast_adx, MAC_ADDR_LENGTH))  //bcast
            {
                //no ack for bcast
                RX_mac_frame rx_frame;
                rx_frame.frame = data_frame_rxing;
                rx_frame.framesize = rx_packet->length;
                m_dcf_callback_context(m_dcf_context,&rx_frame);
            }
        }
        else if(((uint16_t)CTRL_FRAME==protocol_and_type) && ((uint16_t)SUBTYPE_RTS==subtype))  // receive a RTS
        {
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\t\tReceive RTS\n");
            frame_80211_RTS* m_rx_RTS_ptr = (frame_80211_RTS*)rx_packet->buffer;
            set_abs_nav(m_rx_RTS_ptr->dur_id);
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tNAV %d\n", m_rx_RTS_ptr->dur_id);

            if(0 == memcmp(m_rx_RTS_ptr->addr1, dcf_settings.MAC_address,MAC_ADDR_LENGTH))  //to me
            {
                dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\t\tRTS to me\n");
                create_CTS_frame(m_rx_RTS_ptr);
                memcpy (&m_txack_packet.buffer,&m_tx_ack, sizeof(frame_80211_ACK_or_CTS));
                m_txack_packet.length = sizeof(frame_80211_ACK_or_CTS);
#ifdef DCF_LOGGING_ENABLED
                int status = -1;
                status = send_phy_packet(&m_txack_packet);
#else
                send_phy_packet(&m_txack_packet);
#endif
                dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tCTS sent status %d\n",status);
                dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tCTS sent dur %d\n", m_tx_ack.dur_id);
            }
            else
            {
                dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\t\tRTS to others\n");
            }
        }
        else if( ((uint16_t)CTRL_FRAME==protocol_and_type) && ((uint16_t)SUBTYPE_CTS==subtype) )  //receive a CTS
        {
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\t\tReceive CTS\n");
            frame_80211_ACK_or_CTS* m_rx_cts_ptr = (frame_80211_ACK_or_CTS*)rx_packet->buffer;
            set_abs_nav(m_rx_cts_ptr->dur_id);
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tNAV %d\n", m_rx_cts_ptr->dur_id);

            if(0 == memcmp (m_rx_cts_ptr->addr1, dcf_settings.MAC_address,MAC_ADDR_LENGTH))  //to me
            {
                if (dcf_status.need_CTS)
                    dcf_status.has_CTS = true;
            }
        }
    }
    else
    {
        dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tStart probing\n");
        dcf_status.is_probing = true;
        dcf_status.detect_preamble_inside_probe = false;
        //probe for two slots, preempt if detects something
        clock_gettime(CLOCK_MONOTONIC, &rx_probe_end_time);
        rx_probe_end_time.tv_nsec += PROBE_SLOT_NUM*dcf_settings.aSlotTime * NANO_PER_MICRO;
        if (rx_probe_end_time.tv_nsec >= NANO_PER_SEC){
                rx_probe_end_time.tv_nsec -= NANO_PER_SEC;
                rx_probe_end_time.tv_sec +=1;
        }
#ifdef DCF_LOGGING_ENABLED
        gettimeofday(&probe_start, NULL);
#endif
        clock_gettime(CLOCK_MONOTONIC, &rx_probe_curr_time);
        while(is_t1_greater_or_equal_t2(&rx_probe_end_time,&rx_probe_curr_time) &&
              dcf_status.detect_preamble_inside_probe == false)
        {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepstep , NULL);
            clock_gettime(CLOCK_MONOTONIC, &rx_probe_curr_time);
        }
#ifdef DCF_LOGGING_ENABLED
        gettimeofday(&probe_end, NULL);
        rx_timer = (probe_end.tv_sec-probe_start.tv_sec)*MICRO_PER_SEC + probe_end.tv_usec - probe_start.tv_usec;
        dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tprobe for %d us\n", rx_timer);
#endif
        dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tafter probing, receive something %s\n",
                (dcf_status.detect_preamble_inside_probe)?"true":"false");

        dcf_status.is_probing = false;
        if(dcf_status.detect_preamble_inside_probe == false) //if there is nothing after two slots
        {
            if(dcf_status.received_frame == DEDUCE_TO_BE_ACK_OR_CTS )
            {
                if(dcf_status.need_CTS || dcf_status.need_ack)
                    dcf_status.tx_not_preempt = false;
            }
        }

        if((!rx_packet->isFrameDetected)|| (!rx_packet->CRC_valid))
        {
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tRX PHY CRC ERROR, Collision in the air\n" );
            std::cout << "\033[1;31m" << "RX PHY CRC ERROR, Collision in the air" << "\033[0m" << std::endl;
        }
    }
    dcf_log_info((char*)&dcf_logname,"\t\t\t\t\t\tEND OF RX\n");
    return;
}



/**
 * this is a wrapper function to ensure that tx thread and the callback function
 * don't use the same physical layer service at the same time. If this function
 * is busy, 171 is returned.
 */
/*
int Dcf_handler::send_phy_packet(tx_data * phy_packet)
{
    int status = -1;
    int lock_status = pthread_mutex_trylock(&dcf_TX_RX_mutex);

    if( lock_status == 0)
    {
        dcf_log_info((char*)&dcf_logname,"\t\t\tmutex start send\n");
        status = handle.sendPacket(phy_packet);
        dcf_log_info((char*)&dcf_logname,"\t\t\tmutex complete send, status %d\n", status);
    }
    else if (lock_status == EBUSY )
    {
        return SEND_PHY_BUSY;
    }
    else
    {
        dcf_log_error("pthread_mutex_lock error\n");
        exit(EXIT_FAILURE);
    }

    if(pthread_mutex_unlock(&dcf_TX_RX_mutex) != 0)
    {
        dcf_log_error("pthread_mutex_unlock error\n");
        exit(EXIT_FAILURE);
    }
    return status;
}
*/

/**
 * this is a wrapper function to ensure that tx thread and the callback function
 * don't use the same physical layer service at the same time. If this function
 * is busy, 171 is returned.
 */

int Dcf_handler::send_phy_packet(tx_data * phy_packet)
{
    int status = -1;
    if(pthread_mutex_lock(&dcf_TX_RX_mutex) != 0)
    {
        dcf_log_error("pthread_mutex_lock error\n");
        exit(EXIT_FAILURE);
    }

    dcf_log_info((char*)&dcf_logname,"\t\t\tmutex start send\n");
    dcf_status.tx_threading_sending = true;
    status = handle.sendPacket(phy_packet);
    dcf_status.tx_threading_sending= false;
    dcf_log_info((char*)&dcf_logname,"\t\t\tmutex complete send %d\n",status);

    if(pthread_mutex_unlock(&dcf_TX_RX_mutex) != 0)
    {
        dcf_log_error("pthread_mutex_unlock error\n");
        exit(EXIT_FAILURE);
    }
    return status;
}

/** this functions create a RTS frame and store in the private field */
void Dcf_handler::create_RTS_frame (Queue_frame & next_tx_frame, unsigned short MCS_index)
{
    uint16_t frame_ctrl_RTS = 0x0000;
    frame_ctrl_RTS=((frame_ctrl_RTS) << 4| SUBTYPE_RTS ) << 4 | CTRL_FRAME;
    memcpy(&m_tx_RTS.addr1,next_tx_frame.frame.header.addr1,6);
    memcpy(&m_tx_RTS.addr2,dcf_settings.MAC_address,6);
    m_tx_RTS.frame_ctrl = frame_ctrl_RTS;
    m_tx_RTS.dur_id = compute_RTS_DATA_duration(frame_ctrl_RTS, next_tx_frame.framesize, MCS_index);
    memset(m_tx_RTS.fcs, 0x00, CRC_LENGTH);
    return;
}

/** this functions create a CTS frame and store in the private field */
void Dcf_handler::create_CTS_frame (frame_80211_RTS* m_rx_RTS)
{
    memcpy(&m_tx_ack.addr1,m_rx_RTS->addr2,MAC_ADDR_LENGTH);
    uint16_t frame_ctrl_CTS = 0x0000;
    frame_ctrl_CTS=(  (frame_ctrl_CTS ) << 4| SUBTYPE_CTS  ) << 4 | CTRL_FRAME;
    m_tx_ack.frame_ctrl= frame_ctrl_CTS;
    m_tx_ack.dur_id = m_rx_RTS->dur_id - CTS_PHY_TIME - MAC_RX_PROC_TIME;
    memset(m_tx_ack.fcs, 0x00, CRC_LENGTH);
    return;
}

/** this function creates ACK */
void Dcf_handler::create_ACK_frame (frame_80211 * data_frame_rxing)
{
    memcpy(&m_tx_ack.addr1,&data_frame_rxing->header.addr2,MAC_ADDR_LENGTH);
    uint16_t frame_ctrl = 0x0000;
    frame_ctrl=(  (frame_ctrl) << 4 | SUBTYPE_ACK ) << 4 | CTRL_FRAME;
    m_tx_ack.frame_ctrl= frame_ctrl;
    if((data_frame_rxing->header.frame_ctrl & (uint16_t)MORE_FRAG_MASK ) == (uint16_t)MORE_FRAG_MASK)
        m_tx_ack.dur_id =  data_frame_rxing->header.dur_id - ACK_PHY_TIME - MAC_RX_PROC_TIME;
    else                         //final ACK --> 0
        m_tx_ack.dur_id = 0;
    memset(m_tx_ack.fcs, 0x00, CRC_LENGTH);
    return;
}

//compute duration based on frame control
uint16_t Dcf_handler::compute_RTS_DATA_duration (uint16_t frame_control, uint32_t next_frame_length, unsigned short MCS_index)
{
    bool more_frag = false;
    if ((frame_control & (uint16_t)MORE_FRAG_MASK) == (uint16_t)MORE_FRAG_MASK)
        more_frag = true;
    else
        more_frag = false;
    bool is_data_frame = (TYPE_SUBTYPE_MASK & frame_control) == (uint16_t)((SUBTYPE_DATA << 4)| DATA_FRAME );

    if(is_data_frame && !more_frag)             //most common case
    {
        return ACK_PHY_TIME  + MAC_RX_PROC_TIME;
    }
    else                                        //case when RTS or fragmentation happens, duration data needs to be calculated
    {
        uint16_t duration_DATA = 0; //compute expected data duration
        switch(MCS_index){
            case 0:
                duration_DATA = 4.47*next_frame_length +267;
                break;
            case 1:
                duration_DATA = 2.99*next_frame_length +251;
                break;
            case 2:
                duration_DATA = 2.23*next_frame_length +247;
                break;
            case 3:
                duration_DATA = 1.49*next_frame_length +245;
                break;
            case 4:
                duration_DATA = 1.10*next_frame_length +258;
                break;
            case 5:
                duration_DATA = 0.73*next_frame_length + 256;
                break;
            case 6:
                duration_DATA = 0.54 *next_frame_length +258;
                break;
            case 7:
                duration_DATA = 0.48 * next_frame_length +259;
                break;
        }

        if(is_data_frame)
            return duration_DATA + 2*ACK_PHY_TIME + 3 * MAC_RX_PROC_TIME;
        else            //RTS
            return CTS_PHY_TIME + duration_DATA + ACK_PHY_TIME + 3 * MAC_RX_PROC_TIME;
    }
}

/** MAC uses it to set callback function, so that when DCF receives, it can call back*/
void Dcf_handler::dcf_setRxCallback_context(DCF_CallbackFunctionPtr rx_callback_context, void * mac_context)
{
    m_dcf_callback_context = rx_callback_context;
    m_dcf_context = mac_context;
}

/** set retry bit*/
void Dcf_handler::set_retry()
{
    dcf_log_info((char*)&dcf_logname,"set retry bit\n");
    frame_80211 & frame_txing = queue_dcf->front().frame;
    uint16_t frame_ctrl = frame_txing.header.frame_ctrl;
    frame_ctrl = frame_ctrl | (uint16_t)(RETRY_BIT_MASK);
    frame_txing.header.frame_ctrl = frame_ctrl;
}

/** used by RX only to set the NAV value*/
void Dcf_handler::set_abs_nav(uint16_t new_NAV)
{
    pthread_mutex_lock(&read_abs_curr_mutex);
    clock_gettime(CLOCK_MONOTONIC, &curr_abs_time);
    timespec_t1_add_to_t2(&curr_abs_time, new_NAV, &added_nav_time);
    if(is_t1_greater_or_equal_t2(&added_nav_time, &nav_abs_time))
    {
        nav_abs_time.tv_nsec = added_nav_time.tv_nsec;
        nav_abs_time.tv_sec  = added_nav_time.tv_sec;
    }
    pthread_mutex_unlock(&read_abs_curr_mutex);
}

/**
 * used by TX thread to test if absolute time set by NAV is greater than the
 * current absolute time. If YES, TX should not take any action.
 */
bool Dcf_handler::is_abs_nav_greater_abs_curr()
{
    pthread_mutex_lock(&read_abs_curr_mutex);
    clock_gettime(CLOCK_MONOTONIC, &curr_abs_time);
    is_abs_nav_greater_abs_curr_result = is_t1_greater_or_equal_t2(&nav_abs_time, &curr_abs_time);
    pthread_mutex_unlock(&read_abs_curr_mutex);
    return is_abs_nav_greater_abs_curr_result;
}

/**
 * used by TX thread to set DIFS absolute time. It is used before monitoring
 * the actual channel by using the function  bool is_abs_difs_greater_abs_curr,
 * so that the function has a reference to compare with.
 *
 */
void Dcf_handler::set_abs_difs()
{
    pthread_mutex_lock(&read_abs_curr_mutex);
    clock_gettime(CLOCK_MONOTONIC, &curr_abs_time);
    timespec_t1_add_to_t2(&curr_abs_time, dcf_settings.aDIFSTime, &difs_abs_time);
    pthread_mutex_unlock(&read_abs_curr_mutex);
}

/**
 * used by TX thread to check if the pre-set DIFS is greater than the current
 * absolute time
 */
bool Dcf_handler::is_abs_difs_greater_abs_curr()
{
    pthread_mutex_lock(&read_abs_curr_mutex);
    clock_gettime(CLOCK_MONOTONIC, &curr_abs_time);
    is_abs_difs_greater_abs_curr_result = is_t1_greater_or_equal_t2(&difs_abs_time, &curr_abs_time);
    pthread_mutex_unlock(&read_abs_curr_mutex);
    return is_abs_difs_greater_abs_curr_result;
}

/**
 * helper function testing if the first time argument is greater or equal to
 * the second.
 */
bool Dcf_handler::is_t1_greater_or_equal_t2(struct timespec *t1, struct timespec *t2)
{
    if(t1->tv_sec > t2->tv_sec)
        return true;
    else if(t1->tv_sec < t2->tv_sec)
        return false;
    else if(t1->tv_nsec >= t2->tv_nsec )
        return true;
    else
        return false;
}


/** helper function used for adding time to itself*/
void Dcf_handler::timespec_add_us(struct timespec *t, long us)
{
    t->tv_nsec += us*NANO_PER_MICRO;
    if (t->tv_nsec >= NANO_PER_SEC){
            t->tv_nsec -= NANO_PER_SEC;
            t->tv_sec +=1;
    }
}

/**
 * helper function used for adding a relative time(arg2) to a absolute time(arg1)
 * into absolute time(arg3).
 */
void Dcf_handler::timespec_t1_add_to_t2(struct timespec *t1, long us, struct timespec *t2)
{
    t2->tv_nsec = t1->tv_nsec + us * NANO_PER_MICRO;
    t2->tv_sec = t1->tv_sec;
    if(t2->tv_nsec >= NANO_PER_SEC)
    {
        t2->tv_nsec -= NANO_PER_SEC;
        t2->tv_sec += 1;
    }
}

/** set the modulation and coding set*/
void Dcf_handler::set_PHY_TX_MCS(int MCS_index)
{
    dcf_settings.MCS_index = MCS_index;
}

/** to indicate if use FGPA callback or use callback inside DCF*/
void Dcf_handler::use_FPGA_callback_in_DCF(bool is_FPGA_callback_used)
{
    dcf_settings.is_FPGA_callback_used = is_FPGA_callback_used;
}

/**
 * the callback function called by FPGA, this function will deliver data to MAC
 * layer, to do the duplicate elimination.
 */
void Dcf_handler::DCF_RX_callback_FPGA_data(rx_data * data_packet)
{
    //process data and callback to upper layer
    RX_mac_frame rx_frame;
    rx_frame.frame = (frame_80211 *)data_packet->buffer;
    rx_frame.framesize = data_packet->length;
    m_dcf_callback_context(m_dcf_context,&rx_frame);

}

/** callback function used by FPGA to set NAV of DCF */
void Dcf_handler::DCF_RX_callback_FPGA_set_nav(uint16_t rx_nav)
{
    set_abs_nav(rx_nav);
    return;
}

/** callback owned by DCF, is called by PHY when PHY finds a preamble*/
void Dcf_handler::DCF_callback_find_phy_preamble()
{
    dcf_status.detect_preamble= true;
    if(dcf_status.is_probing)
        dcf_status.detect_preamble_inside_probe = true;
}

/**
 * callback owned by DCF, is called by PHY when PHY wants to give header
 * information to DCF
 */
void Dcf_handler::DCF_callback_find_phy_header(int mcs, int length, bool crc_valid)
{
    if(crc_valid)
    {
        if(length == ACK_FRAME_SIZE)      //lock TX until callback for ACK RTS CTS DATA
        {
            set_abs_nav(ACK_DETECT_TO_RECEIVE_TIME);
            dcf_status.received_frame = DEDUCE_TO_BE_ACK_OR_CTS; //ACK or CTS
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\treceive header of ACK/CTS, NAV %d us\n", ACK_DETECT_TO_RECEIVE_TIME);
        }
        else if(length == RTS_FRAME_SIZE)
        {
            set_abs_nav(ACK_DETECT_TO_RECEIVE_TIME);
            dcf_status.received_frame = DEDUCE_TO_BE_RTS; //RTS
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\treceive header of RTS, NAV %d us\n", ACK_DETECT_TO_RECEIVE_TIME);
        }
        else
        {
            uint16_t lock_nav = 0;
            switch (mcs){  // obtained from matlab Detection_estimation.m
                case 0:
                    lock_nav = length * 4.44 + 152;
                    break;
                case 1:
                    lock_nav = length * 2.96  + 140;
                    break;
                case 2:
                    lock_nav = length * 2.21  + 141;
                    break;
                case 3:
                    lock_nav = length * 1.47  + 138;
                    break;
                case 4:
                    lock_nav = length * 1.08  + 149;
                    break;
                case 5:
                    lock_nav = length * 0.71  + 151;
                    break;
                case 6:
                    lock_nav = length * 0.53  + 146;
                    break;
                case 7:
                    lock_nav = length * 0.47  + 147;
                    break;
                default:
                    lock_nav = dcf_settings.ack_timeout;
                    break;
            }
            set_abs_nav(lock_nav);
            dcf_status.received_frame = DEDUCE_TO_BE_DATA;  //DATA
            dcf_log_info((char*)&dcf_logname,"\t\t\t\t\treceive header of DATA, NAV %d us\n", lock_nav);
        }
    }
    else
    {
        set_abs_nav(dcf_settings.ack_timeout);
        dcf_log_info((char*)&dcf_logname,"\t\t\t\t\treceive header with invalid CRC, NAV %d us\n", dcf_settings.ack_timeout);
        std::cout << "\033[1;31m" << "header CRC invalid"<< "\033[0m" << std::endl;
        if(dcf_status.tx_threading_sending)
        {
            dcf_log_info((char*)&dcf_logname,"\t\t\ttxing while receiving\n");
            std::cout << "\033[1;31m" << "txing while receiving"<< "\033[0m" << std::endl;
        }
    }

    if(!dcf_status.detect_preamble)
    {
        dcf_log_info((char*)&dcf_logname,"\t\t\treceive header without sync\n");
        std::cout << "\033[1;31m" << "receive header without sync"<< "\033[0m" << std::endl;
    }
}
