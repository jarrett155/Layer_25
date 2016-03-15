#include "mac_handler.h"
#include "mac_log.h"
#include "dcf_log.h"
#include <sys/time.h>

struct timespec mac_start_time, mac_end_time;
struct timespec mac_operating;
double mac_operating_time =0;
char mac_logname [50]= "default_mac.log";

extern void board_write_data(uint8_t* addr, const char* format, ...); 

int mac_success = 0;
int mac_total_tx_command = 0;

/** Constructor*/
Mac_handler::Mac_handler()
{
    mac_settings.frag_limit = dcf_handler.dcf_settings.frag_limit;     //2346 - sizeof(frame_header_80211_DATA) - MAC_CRC_LENGTH; //page 593,594 

    mac_settings.rate_adapt_switch = false;     //rate adapt default false
    mac_settings.is_FPGA_callback_used = false; //by default, don't use FPGA    
    
    mac_status.transmission_success = -1;
    mac_status.have_queued_frame = false;  
    
    rate_adapt_para.threshold_constant = 5; //rate adapt
    rate_adapt_para.failure_threshold = 2;
    rate_adapt_para.threshold_multiple = 2;
    mac_settings.rate_adapt_switch = false;
    mac_settings.queue_frames = false;
    
    receiver_buffer_length = 0;
    
    mac_settings.is_AP = false;
    
}

/**
 * Start Mac tx thread and open DCF, it also initialize the log file used by 
 * the mac layer, the name is in the form MAC_Board_[mac_addr].
 * @return 
 */
int Mac_handler::start_mac()
{  
    int status = -3;  
    char filename[50];
    uint8_t * addr = mac_settings.my_mac_addr;    
    sprintf(filename,"MAC_Board_%02x%02x%02x%02x%02x%02x.log",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
    strcpy(mac_logname,filename);
#ifdef MAC_RX_CONTENT_LOGGING_ENABLED
    rx_content_log_info(mac_settings.my_mac_addr,"%-18s%-18s%-14s\n","frame ctrl","seq_ctrl","deliver");
#endif    
    //set up and start DCF before activating MAC TX thread
    if(mac_settings.is_AP)
        dcf_handler.dcf_setRxCallback_context(Mac_handler::mac_AP_callback_helper, this);
    else    
        dcf_handler.dcf_setRxCallback_context(Mac_handler::mac_rx_callback_helper, this);                   
        
    dcf_handler.set_myadx(mac_settings.my_mac_addr);       
    dcf_handler.use_FPGA_callback_in_DCF(mac_settings.is_FPGA_callback_used);  
    dcf_handler.set_PHY_TX_MCS(mac_settings.MCS_index);
    dcf_handler.start_dcf();     
    status = pthread_create(&mac_tx_thread, NULL, this->mac_tx_thread_helper, this);
    
    bool dcf_log_enabled = false;
    bool dcf_tx_content_log_enabled = false;
    bool mac_log_enabled = false;
    bool mac_rx_content_log_enabled = false;
    bool display_dcf_transmission_statistics_enabled = false;
    bool display_mac_frame_statistics_enabled = false;    
#ifdef DCF_LOGGING_ENABLED
    dcf_log_enabled = true;
#endif
    
#ifdef DCF_TX_CONTENT_LOGGING_ENABLED
    dcf_tx_content_log_enabled = true;
#endif
    
#ifdef MAC_LOGGING_ENABLED
    mac_log_enabled = true;
#endif
    
#ifdef MAC_RX_CONTENT_LOGGING_ENABLED
    mac_rx_content_log_enabled = true;
#endif
    
#ifdef DISPLAY_DCF_TX_STATISTICS
    display_dcf_transmission_statistics_enabled = true;
#endif
    
#ifdef DISPLAY_MAC_FRAME_STATISTICS
    display_mac_frame_statistics_enabled = true;
#endif
    
    std::cout << "\t\t|" << "my MAC address is                " << "\033[1;32m" << filename << "\033[0m" << std::endl;        
    std::cout << "\t\t|" << "IF FPGA callback is used         " << "\033[1;32m" <<(mac_settings.is_FPGA_callback_used?"True":"False" )<< "\033[0m" << std::endl;    
    std::cout << "\t\t|" << "IF rate adaptation is enabled    " << "\033[1;32m" << (mac_settings.rate_adapt_switch?"True":"False")<< "\033[0m" << std::endl;        
    std::cout << "\t\t|" << "starting MCS mode is             " << "\033[1;32m" <<mac_settings.MCS_index << "\033[0m" << std::endl;
    
    std::cout << "\t\t|" << "MAC fragmentation limit is       " << "\033[1;32m" << mac_settings.frag_limit << "\033[0m" << std::endl;        
    std::cout << "\t\t|" << "MAC/DCF RTS limit is             " << "\033[1;32m" << dcf_handler.dcf_settings.rts_limit << "\033[0m" << std::endl;        
    
    std::cout << "\t\t|" << "MAC RX estimated process time is " << "\033[1;32m" <<MAC_RX_PROC_TIME << " us" << "\033[0m" << std::endl;
    std::cout << "\t\t|" << "DCF ACK timeout is               " << "\033[1;32m" <<dcf_handler.dcf_settings.ack_timeout << " us" << "\033[0m" << std::endl;
    std::cout << "\t\t|" << "DCF DIFS time is                 " << "\033[1;32m" <<dcf_handler.dcf_settings.aDIFSTime << " us"<< "\033[0m" << std::endl;
    std::cout << "\t\t|" << "DCF slot time is                 " << "\033[1;32m" <<dcf_handler.dcf_settings.aSlotTime << " us"<< "\033[0m" << std::endl;
    std::cout << "\t\t|" << "PHY estimated ACK time is        " << "\033[1;32m" <<ACK_PHY_TIME << " us" << "\033[0m" << std::endl;   
    std::cout << "\t\t|" << "---------------------------------" << "\033[1;32m" << "\033[0m" << std::endl;       
    std::cout << "\t\t|" << "DCF activity log enabled         " << "\033[1;32m" <<(dcf_log_enabled?"True":"False" ) << "\033[0m" << std::endl;   
    std::cout << "\t\t|" << "DCF TX Content log enabled       " << "\033[1;32m" <<(dcf_tx_content_log_enabled?"True":"False" ) << "\033[0m" << std::endl;   
    std::cout << "\t\t|" << "Mac activity log enabled         " << "\033[1;32m" <<(mac_log_enabled?"True":"False" ) << "\033[0m" << std::endl;   
    std::cout << "\t\t|" << "Mac rx content log enabled       " << "\033[1;32m" <<(mac_rx_content_log_enabled?"True":"False" ) << "\033[0m" << std::endl;
    std::cout << "\t\t|" << "Display DCF TX statistics        " << "\033[1;32m" <<(display_dcf_transmission_statistics_enabled?"True":"False" ) << "\033[0m" << std::endl;
    std::cout << "\t\t|" << "Display MAC frame statistics     " << "\033[1;32m" <<(display_mac_frame_statistics_enabled?"True":"False" ) << "\033[0m" << std::endl;
    std::cout << "\t\t|" << "---------------------------------" << "\033[1;32m" << "\033[0m" << std::endl;   
    std::cout << "\t\t|" << "Interframe idle unit is          " << "\033[1;32m" << FRAME_IDLE_UNIT << " us" << "\033[0m" << std::endl;   
    mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\t station starts\n");

    return status;
}

/**
 * This function is called by network layer to send the upper layer packet.
 * @param payload   a pointer points to upper layer data
 * @param des_addr  a pointer points to its address
 * @param length    uint16_t to store the total length of the upper layer frame
 * @return 0 is returned when it is sent success;
 *         -1 is returned when it is a failure.
 *         -2 is returned when previous frame is unsent.
 */
int Mac_handler::send_frame(uint8_t* payload, uint8_t* des_addr, uint16_t length)
{       
    pthread_mutex_lock(&MAC_queue);
    clock_gettime(CLOCK_MONOTONIC, &mac_start_time);
    mac_log_info((char*)&mac_logname,"MAC start to send datagram\n");
    mac_status.curr_frag_number = 0;
    if (!mac_status.have_queued_frame)
    {   
        
        destination_mac_addr = des_addr;        
        datagram = payload;        
        data_length = length; 
        mac_status.have_queued_frame = true;         
        pthread_cond_broadcast(&MAC_tx_request);
    }
    else
    {
        mac_log_error("previous frame still unsent\n");        
        mac_status.transmission_success =  -2;
    }   
    mac_log_info((char*)&mac_logname,"MAC is working on it\n");    
    pthread_mutex_lock(&MAC_lock);
    while((mac_status.have_queued_frame))  //if dcf have a frame to send    
        pthread_cond_wait(&MAC_ready,&MAC_lock);
    pthread_mutex_unlock(&MAC_lock);
#ifdef DISPLAY_MAC_FRAME_STATISTICS
    clock_gettime(CLOCK_MONOTONIC, &mac_end_time);
    mac_operating.tv_sec = mac_end_time.tv_sec - mac_start_time.tv_sec;
    mac_operating.tv_nsec = mac_end_time.tv_nsec - mac_start_time.tv_nsec;
    while (mac_operating.tv_nsec > 1000000000) 
    {
            mac_operating.tv_sec++;
            mac_operating.tv_nsec -= 1000000000;
    }
    while (mac_operating.tv_nsec < 0) 
    {
            mac_operating.tv_sec--;
            mac_operating.tv_nsec += 1000000000;
    }   
    mac_operating_time += mac_operating.tv_sec + mac_operating.tv_nsec/1000000000.0; 
    if(mac_total_tx_command%100 == 0) {
        printf("\t\t\t#%d frame MAC time: %-7.3fs, suc frm %d, ratio %.2f%%\n\n",mac_total_tx_command, mac_operating_time,mac_success,100*mac_success/(double)mac_total_tx_command); //in case log is disabled but time is displayed        
    }
    mac_log_info((char*)&mac_logname,"Data delivered, MAC operating time: %-8.3f s\n",mac_operating_time);
#endif
    pthread_mutex_unlock(&MAC_queue);
    return mac_status.transmission_success;
}

/**
 * The function handlers network layer packet and perform segmentation according
 * to its length. All the fragmented frame are pushed into a queue which waits 
 * for DCF to transmit them.
 * @return 
 */
void * Mac_handler::MAC_TX_thread()
{
    while(1)
    {
        
        if (!mac_settings.is_AP)
        {                    
            if(mac_status.have_queued_frame)
            {
                //determine max number of fragments, limit is compared with the 
                //entire frame including header and fcs
                uint16_t max_frag_number = (data_length -1) / (mac_settings.frag_limit-sizeof(frame_header_80211_DATA)-CRC_LENGTH); 
                for (mac_status.curr_frag_number = 0 ; mac_status.curr_frag_number <= max_frag_number; mac_status.curr_frag_number++)
                {                
                    uint32_t frame_length;
                    if (mac_settings.rate_adapt_switch)
                    {
                        decide_record();                    
                    }
                    //frame_length = create_data_frame(max_frag_number);  
                    frame_length = create_to_AP_frame(datagram,data_length, destination_mac_addr, ap_mac_addr);                
                    queue_frame(&m_tx_frame, frame_length);
                }                         

                mac_log_info((char*)&mac_logname,"ALL %d frames are queued\n", max_frag_number+1);
                if(mac_settings.rate_adapt_switch)   //queue has all fragmented frame 
                {                         
                    rate_adaptation();
                    mac_status.transmission_success = dcf_handler.send_frame(&queue, mac_status.curr_record); 
                }
                else
                {           
                    mac_status.transmission_success = dcf_handler.send_frame(&queue); 
                }
                mac_total_tx_command++;
                mac_status.seq_number++;
                switch (mac_status.transmission_success){
                    case 0:
                        mac_success++;
                        mac_log_info((char*)&mac_logname,"SUCCESS %d/%d\n", mac_success,mac_total_tx_command);
                        break;
                    default:
                        break;
                }
                /* collect info */            
                if(mac_settings.rate_adapt_switch)
                {                  
                    if (mac_status.transmission_success == 0)  //corresponds success assignment based on send_frame in dcf 
                    {
                        mac_status.curr_record->consec_success++;  //rate adapt                        
                        mac_status.curr_record->consec_failure=0;
                    }
                    else
                    {
                        mac_status.curr_record->consec_failure++;
                        mac_status.curr_record->consec_success = 0;
                        if(mac_status.curr_record->is_first)         //rate adapt
                        {
                           mac_status.curr_record->is_first_retried = true;
                        }
                    }
                }

                mac_status.have_queued_frame = false;
                pthread_cond_broadcast(&MAC_ready);
            }
            else
            { 
                pthread_mutex_lock(&MAC_tx_request_lock);
                pthread_cond_wait(&MAC_tx_request,&MAC_tx_request_lock);
                pthread_mutex_unlock(&MAC_tx_request_lock);                        
            }
        }
        else
        {
            mac_log_info((char*)&mac_logname,"AP is checking\n");
            if(!queue.empty())
            {
                mac_log_info((char*)&mac_logname,"before send relay frame\n");
                mac_status.transmission_success = dcf_handler.send_frame(&queue); 
            }
            else
            {
                pthread_mutex_lock(&MAC_tx_request_lock);
                pthread_cond_wait(&MAC_tx_request,&MAC_tx_request_lock);
                pthread_mutex_unlock(&MAC_tx_request_lock);                 
            }
        }
    }
    pthread_exit(NULL);
}

/** this function push the data frame into the queue */
void Mac_handler::queue_frame(frame_80211* frame, uint32_t len)
{
    Queue_frame queue_frame;
    queue_frame.frame = *frame;    
    queue_frame.framesize = len;
    queue.push(queue_frame); 
    mac_log_info((char*)&mac_logname,"push a frame into queue\n");
    return;
}

/**
 * this function creates create data frames. It uniformly treats fragmented and 
 * non-fragmented frame. Under the case of fragmentation, after the previous 
 * frame is transmitted, the function can locate where the next piece of data is
 * and create the next frame and increment the next data pointer location.
 * @param    max_frag_number
 * @return   length of the entire DATA frame 
 */
uint32_t Mac_handler::create_data_frame (uint16_t max_frag_number)
{
    uint8_t* payload_ptr = NULL;
    uint16_t frame_ctrl = 0x0000;
    uint16_t seq_ctrl = 0x0000;
    uint16_t mask = 0x0001;
    uint8_t BBSID[6];
    uint32_t payload_length = 0, next_payload_length = 0;
    uint32_t frame_length = 0, next_frame_length = 0;
    BBSID[0] = 0x11;  //not implemented yet
    bool more_frag = false;
    
    if ( max_frag_number != mac_status.curr_frag_number ) //decide if more_frag
    {
        more_frag = true;    
    }
    if (more_frag)           
    {   
        frame_ctrl=((frame_ctrl | mask<<Frame_ctrl(moreFrags) ) << 4 | SUBTYPE_DATA ) << 4 | DATA_FRAME;    
    }
    else
    {
        frame_ctrl=((frame_ctrl) << 4 | SUBTYPE_DATA ) << 4 | DATA_FRAME; 
    } 
    
    m_tx_frame.header.frame_ctrl = frame_ctrl;       
    memcpy(m_tx_frame.header.addr1,destination_mac_addr,MAC_ADDR_LENGTH);
    memcpy(m_tx_frame.header.addr2,mac_settings.my_mac_addr,MAC_ADDR_LENGTH);
    memcpy(m_tx_frame.header.addr3,BBSID,MAC_ADDR_LENGTH);
    
    seq_ctrl = (uint16_t)((mac_status.seq_number%SEQUENCE_NUM_MOD) << 4 | mac_status.curr_frag_number);   
    m_tx_frame.header.seq_ctl = seq_ctrl;
        
    if (max_frag_number != 0 ) // decide if this frame is a fragmented one
    {
        payload_ptr = datagram + mac_status.curr_frag_number * (mac_settings.frag_limit-sizeof(frame_header_80211_DATA)-CRC_LENGTH);
        if(mac_status.curr_frag_number == max_frag_number)  // last fragment 
        {
            payload_length = data_length - mac_status.curr_frag_number*(mac_settings.frag_limit-sizeof(frame_header_80211_DATA)-CRC_LENGTH);  
            next_payload_length = 0;
        }
        else
        {
            payload_length = mac_settings.frag_limit - sizeof(frame_header_80211_DATA) - CRC_LENGTH ; 
            if(mac_status.curr_frag_number == max_frag_number - 1)
                next_payload_length = data_length - (mac_status.curr_frag_number+1)*(mac_settings.frag_limit-sizeof(frame_header_80211_DATA)-CRC_LENGTH); 
            else
                next_payload_length = payload_length;
        }
    }
    else
    {
        payload_ptr = datagram;
        payload_length = data_length;
        next_payload_length = 0;
    }
    frame_length = sizeof(frame_header_80211_DATA) + payload_length + CRC_LENGTH;
    next_frame_length = sizeof(frame_header_80211_DATA) + next_payload_length + CRC_LENGTH;
    mac_log_info((char*)&mac_logname,"#%d frame: length %d, next: length %d\n",mac_status.curr_frag_number,frame_length, next_frame_length ); 
    
    if(mac_settings.rate_adapt_switch)
        m_tx_frame.header.dur_id = dcf_handler.compute_RTS_DATA_duration(frame_ctrl, next_frame_length, mac_status.curr_record->current_rate); 
    else
        m_tx_frame.header.dur_id = dcf_handler.compute_RTS_DATA_duration(frame_ctrl, next_frame_length, mac_settings.MCS_index); 
    memcpy(m_tx_frame.buffer_and_fcs, payload_ptr, payload_length);
    m_tx_frame.buffer_and_fcs[0] = datagram[0];                                                     //debug to show frame id      
    memset((uint8_t*)&m_tx_frame + frame_length - CRC_LENGTH, 0x00, CRC_LENGTH);                       //set last four bytes to zero
#ifdef MAC_LOGGING_ENABLED
    uint8_t* addr = destination_mac_addr;
    mac_log_info((char*)&mac_logname,"Destination address is  0x%02x:%02x:%02x:%02x:%02x:%02x. \tframe size %d\n", 
            addr[0],addr[1],addr[2],addr[3],addr[4],addr[5],frame_length);
#endif
    return frame_length; //total frame length include
}

/** this function set MAC address */
int Mac_handler::set_myadx(uint8_t adx[6])
{
    memcpy(&mac_settings.my_mac_addr,adx,6); 
    return 0;
}

/** this function is called by DCF so that MAC can do some operation based on it*/
void Mac_handler::mac_RX_callback(RX_mac_frame * rx_frame)
{   
    mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\treceive type DATA of size %d in connected mode\n", rx_frame->framesize);
    uint16_t frame_ctrl = rx_frame->frame->header.frame_ctrl;
    uint64_t addr2 = 0x0000000000000000;
    uint16_t curr_seq_ctrl, prev_seq_ctrl, retry_bit_mask = 0x0800;
    curr_seq_ctrl = rx_frame->frame->header.seq_ctl;
    memcpy(&addr2, rx_frame->frame->header.addr2, MAC_ADDR_LENGTH);
    uint16_t retry_bit = retry_bit_mask & (rx_frame->frame->header.frame_ctrl);
    std::map<uint64_t, uint16_t>::iterator iter;
    bool pair_just_created = false;
    if (mac_status.seq_ctrl_map.find(addr2) == mac_status.seq_ctrl_map.end())
    {            
        mac_status.seq_ctrl_map[addr2] = *(new uint16_t);
        pair_just_created = true;
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tcreating a map for this addr\n");
    } 
    else
    {
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tmap finds the pair\n");
    }
    iter = mac_status.seq_ctrl_map.find(addr2);
    if(!pair_just_created)
        prev_seq_ctrl = iter->second;
#ifdef MAC_RX_CONTENT_LOGGING_ENABLED  
    rx_content_print_bit(mac_settings.my_mac_addr,frame_ctrl);                        
    rx_content_log_info(mac_settings.my_mac_addr,"  ");
    rx_content_print_bit(mac_settings.my_mac_addr,curr_seq_ctrl); 
    rx_content_log_info(mac_settings.my_mac_addr,"  ");
    rx_content_log_info(mac_settings.my_mac_addr,"\n");
#endif
    uint16_t prev_seq_num = (prev_seq_ctrl & 0xFFF0) >> 4;
    uint16_t curr_seq_num =  (curr_seq_ctrl & 0xFFF0) >> 4;        
    if ( pair_just_created||prev_seq_num != curr_seq_num)
    {         
        receiver_buffer_length = 0;
    }
    mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tduplication testing:\n");
    if (pair_just_created||((retry_bit==retry_bit_mask) && (curr_seq_ctrl != prev_seq_ctrl)) || retry_bit!=retry_bit_mask)
    {
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tNot duplicate or not retried\n");
        uint32_t datagram_length = rx_frame->framesize - CRC_LENGTH - sizeof(frame_header_80211_DATA);
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\ttotal frm length %d, get the length %d\n",rx_frame->framesize,datagram_length);
        memcpy(receiver_buffer + receiver_buffer_length ,rx_frame->frame->buffer_and_fcs, datagram_length);
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tcopy data\n");
        receiver_buffer_length += datagram_length;
        iter->second = curr_seq_ctrl;               
    }
    else
    {
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tdetect Duplicate\n");
        return;
    }
    
    uint16_t more_frag_mask = MORE_FRAG_MASK;     
    if (((more_frag_mask & frame_ctrl) == more_frag_mask)) //if more frag
    {
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tmore frag\n");
        return;         
    }
    else
    {
        deliver_data.data = receiver_buffer;
        deliver_data.data_size = receiver_buffer_length;
        deliver_data.source_addr = rx_frame->frame->header.addr2;
        deliver_data.seq_num = curr_seq_num;
        if (mac_settings.queue_frames)
        {
            mac_settings.frame_queue->push(deliver_data);
        }
        mac_rx_call_up(&deliver_data);     
    #ifdef MAC_RX_CONTENT_LOGGING_ENABLED        
        rx_content_log_info(mac_settings.my_mac_addr,"\t\t\t\t\t%-8d  size %d\n",receiver_buffer[0],deliver_data.data_size); 
    #endif       
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tdeliver data %d\n",deliver_data.data_size);
        receiver_buffer_length = 0;
        return;
    }  
}

void Mac_handler::mac_AP_callback(RX_mac_frame* rx_frame)
{
    mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\treceive type DATA of size %d in connected mode\n", rx_frame->framesize);
    uint16_t frame_ctrl = rx_frame->frame->header.frame_ctrl;
    uint64_t addr2 = 0x0000000000000000;
    uint16_t curr_seq_ctrl, prev_seq_ctrl, retry_bit_mask = 0x0800;
    curr_seq_ctrl = rx_frame->frame->header.seq_ctl;
    memcpy(&addr2, rx_frame->frame->header.addr2, MAC_ADDR_LENGTH);
    uint16_t retry_bit = retry_bit_mask & (rx_frame->frame->header.frame_ctrl);
    std::map<uint64_t, uint16_t>::iterator iter;
    bool pair_just_created = false;
    if (mac_status.seq_ctrl_map.find(addr2) == mac_status.seq_ctrl_map.end())
    {            
        mac_status.seq_ctrl_map[addr2] = *(new uint16_t);
        pair_just_created = true;
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tcreating a map for this addr\n");
    } 
    else
    {
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tmap finds the pair\n");
    }
    iter = mac_status.seq_ctrl_map.find(addr2);
    if(!pair_just_created)
        prev_seq_ctrl = iter->second;
#ifdef MAC_RX_CONTENT_LOGGING_ENABLED  
    rx_content_print_bit(mac_settings.my_mac_addr,frame_ctrl);                        
    rx_content_log_info(mac_settings.my_mac_addr,"  ");
    rx_content_print_bit(mac_settings.my_mac_addr,curr_seq_ctrl); 
    rx_content_log_info(mac_settings.my_mac_addr,"  ");
    rx_content_log_info(mac_settings.my_mac_addr,"\n");
#endif
    uint16_t prev_seq_num = (prev_seq_ctrl & 0xFFF0) >> 4;
    uint16_t curr_seq_num =  (curr_seq_ctrl & 0xFFF0) >> 4;        
    if ( pair_just_created||prev_seq_num != curr_seq_num)
    {         
        receiver_buffer_length = 0;
    }
    mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tduplication testing:\n");
    if (pair_just_created||((retry_bit==retry_bit_mask) && (curr_seq_ctrl != prev_seq_ctrl)) || retry_bit!=retry_bit_mask)
    {
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tNot duplicate or not retried\n");
        uint32_t datagram_length = rx_frame->framesize - CRC_LENGTH - sizeof(frame_header_80211_DATA);
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\ttotal frm length %d, get the length %d\n",rx_frame->framesize,datagram_length);
        memcpy(receiver_buffer + receiver_buffer_length ,rx_frame->frame->buffer_and_fcs, datagram_length);
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tcopy data\n");
        receiver_buffer_length += datagram_length;
        iter->second = curr_seq_ctrl;               
    }
    else
    {
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tdetect Duplicate\n");
        return;
    }
    
    uint16_t more_frag_mask = MORE_FRAG_MASK;     
    if (((more_frag_mask & frame_ctrl) == more_frag_mask)) //if more frag
    {
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\tmore frag\n");
        return;         
    }
    else
    {
        mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\t processing relay data\n");
        uint16_t relay_frame_length = 0;
        relay_frame_length = create_from_AP_frame(rx_frame->frame->buffer_and_fcs, rx_frame->framesize - CRC_LENGTH - sizeof(frame_header_80211_DATA),
               rx_frame->frame->header.addr3 , rx_frame->frame->header.addr2);
        push_relay_frame(&relay_frame, relay_frame_length);
        pthread_cond_broadcast(&MAC_tx_request);
        return;                                
    }
    
}

void Mac_handler::push_relay_frame(frame_80211* frame, uint16_t length)
{
    mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\t AP push relay data\n");
    pthread_mutex_lock(&MAC_queue);
    queue_frame(frame, length);
    pthread_mutex_unlock(&MAC_queue);        
    mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\t queue size %d\n", queue.size());
    return;    
}


int Mac_handler::send_frame(uint8_t* payload, uint8_t* des_addr, uint8_t* AP_addr, uint16_t length)
{    
    pthread_mutex_lock(&MAC_queue);
    clock_gettime(CLOCK_MONOTONIC, &mac_start_time);
    mac_log_info((char*)&mac_logname,"MAC start to send datagram\n");
    mac_status.curr_frag_number = 0;
    if (!mac_status.have_queued_frame)
    {   
        ap_mac_addr = AP_addr; 
        destination_mac_addr = des_addr;        
        datagram = payload;        
        data_length = length; 
        mac_status.have_queued_frame = true;         
        pthread_cond_broadcast(&MAC_tx_request);
    }
    else
    {
        mac_log_error("previous frame still unsent\n");        
        mac_status.transmission_success =  -2;
    }   
    mac_log_info((char*)&mac_logname,"MAC is working on it\n");    
    pthread_mutex_lock(&MAC_lock);
    while((mac_status.have_queued_frame))  //if dcf have a frame to send    
        pthread_cond_wait(&MAC_ready,&MAC_lock);
    pthread_mutex_unlock(&MAC_lock);
#ifdef DISPLAY_MAC_FRAME_STATISTICS
    clock_gettime(CLOCK_MONOTONIC, &mac_end_time);
    mac_operating.tv_sec = mac_end_time.tv_sec - mac_start_time.tv_sec;
    mac_operating.tv_nsec = mac_end_time.tv_nsec - mac_start_time.tv_nsec;
    while (mac_operating.tv_nsec > 1000000000) 
    {
            mac_operating.tv_sec++;
            mac_operating.tv_nsec -= 1000000000;
    }
    while (mac_operating.tv_nsec < 0) 
    {
            mac_operating.tv_sec--;
            mac_operating.tv_nsec += 1000000000;
    }   
    mac_operating_time += mac_operating.tv_sec + mac_operating.tv_nsec/1000000000.0; 
    if(mac_total_tx_command%100 == 0) {
        printf("\t\t\t#%d frame MAC time: %-7.3fs, suc frm %d, ratio %.2f%%\n\n",mac_total_tx_command, mac_operating_time,mac_success,100*mac_success/(double)mac_total_tx_command); //in case log is disabled but time is displayed        
    }
    mac_log_info((char*)&mac_logname,"Data delivered, MAC operating time: %-8.3f s\n",mac_operating_time);
#endif
    pthread_mutex_unlock(&MAC_queue);
    return mac_status.transmission_success;         
}

uint16_t Mac_handler::create_from_AP_frame(uint8_t* data,uint16_t length, uint8_t* DA, uint8_t* SA)
{    
    mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\t AP creates from AP frame\n");
    uint16_t frame_ctrl = 0x0000;
    uint16_t mask = 0x0001;    
    uint16_t seq_ctrl = 0x0000;
    
    uint8_t* payload_ptr = NULL;        
    uint32_t payload_length = 0 ;
    uint32_t frame_length = 0;
                
    frame_ctrl=((frame_ctrl | mask<<Frame_ctrl(fromDs) ) << 4 | SUBTYPE_DATA ) << 4 | DATA_FRAME;
    relay_frame.header.frame_ctrl = frame_ctrl;
    memcpy(relay_frame.header.addr1, DA, MAC_ADDR_LENGTH);
    memcpy(relay_frame.header.addr2, mac_settings.my_mac_addr, MAC_ADDR_LENGTH);
    memcpy(relay_frame.header.addr3, SA, MAC_ADDR_LENGTH);
    seq_ctrl = (uint16_t)((mac_status.seq_number%SEQUENCE_NUM_MOD) << 4 | mac_status.curr_frag_number);
    relay_frame.header.seq_ctl = seq_ctrl;
    
    //don't consider fragmented case
    payload_ptr = data;
    payload_length = length;
    
    
    frame_length = sizeof(frame_header_80211_DATA) + payload_length + CRC_LENGTH;    
    mac_log_info((char*)&mac_logname,"#%d frame: length %d\n",mac_status.curr_frag_number,frame_length ); 
    
    if(mac_settings.rate_adapt_switch)
        relay_frame.header.dur_id = dcf_handler.compute_RTS_DATA_duration(frame_ctrl, 0, mac_status.curr_record->current_rate); 
    else
        relay_frame.header.dur_id = dcf_handler.compute_RTS_DATA_duration(frame_ctrl, 0, mac_settings.MCS_index); 
    
    memcpy(relay_frame.buffer_and_fcs, data, length);
    memcpy(relay_frame.buffer_and_fcs, payload_ptr, payload_length);    
    memset((uint8_t*)&relay_frame + frame_length - CRC_LENGTH, 0x00, CRC_LENGTH);                       //set last four bytes to zero
    return frame_length;        
}

int Mac_handler::get_seq_num()
{
    return mac_status.seq_number%SEQUENCE_NUM_MOD;
}

uint16_t Mac_handler::create_to_AP_frame(uint8_t* data,uint16_t length, uint8_t* DA, uint8_t* AP)
{
    mac_log_info((char*)&mac_logname,"\t\t\t\t\t\t\t client creates to AP frame\n");
    uint16_t frame_ctrl = 0x0000;
    uint16_t mask = 0x0001;    
    uint16_t seq_ctrl = 0x0000;
    
    uint8_t* payload_ptr = NULL;        
    uint32_t payload_length = 0 ;
    uint32_t frame_length = 0;
                
    frame_ctrl=((frame_ctrl | mask<<Frame_ctrl(toDs) ) << 4 | SUBTYPE_DATA ) << 4 | DATA_FRAME;
    m_tx_frame.header.frame_ctrl = frame_ctrl;
    memcpy(m_tx_frame.header.addr1, DA, MAC_ADDR_LENGTH);
    memcpy(m_tx_frame.header.addr2, mac_settings.my_mac_addr, MAC_ADDR_LENGTH);
    memcpy(m_tx_frame.header.addr3, AP, MAC_ADDR_LENGTH);
    seq_ctrl = (uint16_t)((mac_status.seq_number%SEQUENCE_NUM_MOD) << 4 | mac_status.curr_frag_number);
    m_tx_frame.header.seq_ctl = seq_ctrl;

    
    //don't consider fragmented case
    payload_ptr = data;
    payload_length = length;
    
    
    frame_length = sizeof(frame_header_80211_DATA) + payload_length + CRC_LENGTH;    
    mac_log_info((char*)&mac_logname,"#%d frame: length %d\n",mac_status.curr_frag_number,frame_length ); 
    
    if(mac_settings.rate_adapt_switch)
        m_tx_frame.header.dur_id = dcf_handler.compute_RTS_DATA_duration(frame_ctrl, 0, mac_status.curr_record->current_rate); 
    else
        m_tx_frame.header.dur_id = dcf_handler.compute_RTS_DATA_duration(frame_ctrl, 0, mac_settings.MCS_index); 
    
    memcpy(m_tx_frame.buffer_and_fcs, data, length);
    memcpy(m_tx_frame.buffer_and_fcs, payload_ptr, payload_length);    
    memset((uint8_t*)&m_tx_frame + frame_length - CRC_LENGTH, 0x00, CRC_LENGTH);                       //set last four bytes to zero
    return frame_length;    
}

void Mac_handler::set_AP(bool is_set_to_AP)
{
    mac_settings.is_AP = is_set_to_AP;
}

/** setting callback context and callback function to MAC, set by MAC*/
void Mac_handler::mac_setRxCallback_context(MAC_CallbackFunctionPtr rx_callback_context, void * mac_upper_context)
{
    m_upper_mac_callback_helper = rx_callback_context;
    m_upper_mac_context = mac_upper_context;
}

/** setting callback function to MAC, set by MAC*/
void Mac_handler::mac_setRxCallback(void (*rx_callback) (struct RX_datagram* data)){
    this->mac_rx_call_up = rx_callback;
}

/** decide which record it nemac_STA_receivereds to use */
void Mac_handler::decide_record()
{
    uint64_t addr_key = 0000000000000000;
    memcpy(&addr_key, destination_mac_addr, sizeof(m_tx_frame.header.addr1));
    //if no such element, create a mapping 
    if (mac_status.rate_map.find(addr_key) == mac_status.rate_map.end())
    { 
        mac_status.rate_map[addr_key] = *(new Rate_record);
        mac_status.rate_map[addr_key].consec_failure = 0;
        mac_status.rate_map[addr_key].consec_success = 0;
        mac_status.rate_map[addr_key].curr_threshold = rate_adapt_para.threshold_constant;
        mac_status.rate_map[addr_key].is_first = false;
        mac_status.rate_map[addr_key].is_first_retried = false;
    }    
    std::map<uint64_t,Rate_record>::iterator iter;
    iter = mac_status.rate_map.find(addr_key);
    mac_status.curr_record = &(iter->second);
}

/** It enables rate adaptation*/
void Mac_handler::enable_rate_adaptation()
{
    mac_settings.rate_adapt_switch = true;
}

/** It disables rate adaptation*/
void Mac_handler::disable_rate_adaptation()
{
    mac_settings.rate_adapt_switch = false;
}

/** rate adaptation */
void Mac_handler::rate_adaptation()
{   
    Rate_record* curr_record = mac_status.curr_record;
    if((curr_record->is_first)) //imply previous success
    {
        if(curr_record->is_first_retried)
        {
            curr_record->curr_threshold = rate_adapt_para.threshold_multiple * (curr_record->curr_threshold);  
            curr_record->current_rate--;
            curr_record->is_first_retried = false;
        } 
        else
        {
            curr_record->curr_threshold = rate_adapt_para.threshold_constant;    
        }
        curr_record->is_first = false;    
    }            
    else  
    {
        if(curr_record->consec_success >= curr_record->curr_threshold)
        {   
            if(curr_record->current_rate < MAX_RATE_MODE)
            {
                (curr_record->current_rate)++;
            }

            curr_record->consec_success = 0;
            if (curr_record->current_rate != MAX_RATE_MODE) //if in the max mode, no need to increase, 
            {                                          //so no need to set is_first
                curr_record->is_first = true;
            }
        }

        if(curr_record->consec_failure >= rate_adapt_para.failure_threshold)     
        {
            if(curr_record->current_rate > MIN_RATE_MODE)
            {
                (curr_record->current_rate)--;
            }
            curr_record->curr_threshold = rate_adapt_para.threshold_constant;
            curr_record->consec_failure = 0;
        }
    }  
    mac_log_info((char*)&mac_logname,"rate is changed to %d\n",curr_record->current_rate);
}

/** to indicate if use FGPA callback*/
void Mac_handler::use_FPGA_callback_in_DCF(bool is_FPGA_callback_used)
{
    mac_settings.is_FPGA_callback_used = is_FPGA_callback_used;
}

/** set starting modulation and coding set*/
void Mac_handler::set_PHY_TX_MCS(unsigned short MCS_index)
{
    mac_settings.MCS_index = MCS_index;
}

 void Mac_handler::set_frame_queue(std::queue<RX_datagram> *queue_for_frames)
 {
     mac_settings.frame_queue = queue_for_frames;
     mac_settings.queue_frames = true;
 }
