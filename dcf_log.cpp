#include <dcf_log.h>
#include <stdio.h>
#include <stdarg.h>
#ifdef DCF_LOGGING_ENABLED
#include <time.h>

static dcf_log_level filter_level = DCF_LOG_LEVEL_VERBOSE;
static bool logfile_cleared = false;

/** this function is used with MARCO to write the desired message on the screen*/
void dcf_log_write(dcf_log_level level, const char *caller, const char *format, ...)
{
    /* Only process this message if its level exceeds the current threshold */
    if (level >= filter_level)
    {
        va_list args;
        char s[1000];
        
        /* Write the log message */
        va_start(args, format);
        vsprintf(s, format, args);
        printf("[%s]%s", caller, s);
        
        //vfprintf(stderr, format, args);
        va_end(args);
    }
}

/** 
 * this function is used with MARCO to write data in a certain file specified by the
 * filename in the argument. The file will be automatically cleared when this is a 
 * new process/test. 
 */
void dcf_log_file_write(const char* filename,dcf_log_level level, const char *caller, 
        const char *format, ...)
{
    if(!logfile_cleared){
        FILE* fp = fopen(filename, "w");
        if(fp != NULL){
            time_t rawtime;
            time ( &rawtime );
            fprintf(fp, "DCF Log File: [%s] \n", ctime (&rawtime));
            fclose(fp);
        }
        logfile_cleared = true;
    }

    FILE* fp = fopen(filename, "a+");
    if(fp!=NULL){
        /* Only process this message if its level exceeds the current threshold */
        if (level >= filter_level)
        {
            va_list args;
            char s[1000];

            /* Write the log message */
            va_start(args, format);
            vsprintf(s, format, args);
            fprintf(fp, "[%s]%s", caller, s);
            va_end(args);
        }
        fclose(fp);
    }
}
#endif

#ifdef DCF_TX_CONTENT_LOGGING_ENABLED
/**
 * this function is used to write detailed transmission information into a log
 * file, which includes transmission results(success/failure), RTT in the DCF
 * the sending time using by physical layer, source address and destination 
 * address, more importantly, the full content of frame control and sequence
 * control. 
 * @param addr
 * @param format
 * @param ...
 */
#include <time.h>
static bool tx_content_logfile_cleared = false;
void tx_content_log_info(uint8_t* addr, const char* format, ...)
{
    char filename[50];   
    sprintf(filename, "TX_CONTENT_%02x%02x%02x%02x%02x%02x.log", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    if(!tx_content_logfile_cleared){
        fclose(fopen(filename, "w"));
        tx_content_logfile_cleared = true;
    }      
    va_list args;
    va_start (args, format);   
    FILE* stream;
    stream = fopen(filename,"a+");     
    vfprintf (stream, format, args);
    va_end (args); 
    fclose(stream);  
}   

/**
 * this function is used with tx_content_log_info, this function is used to 
 * print out every bit of a uint16_t type, the LSB is on the right side.
 * @param addr
 * @param bit_to_print
 */
void tx_content_print_bit(uint8_t *addr, uint16_t bit_to_print)
{
    uint16_t mask = 0x1;
    uint16_t selectBit = 0x0; 
    uint16_t result = 0x0;
    //backward order
    for (int i = 0; i< 16 ; i++)
    {
        selectBit = (((mask<<i) & bit_to_print)>>i) & mask;
        result = (result << 1) | selectBit;  
    } 
    //print out bit
    for (int i = 0; i< 16;i++)
    {
        tx_content_log_info(addr, "%d", (((mask<<i) & result)>>i) & mask);
    } 
}
#endif

/** 
 * this function is a debug function, it is called by its name directly, the 
 * function can print the desired the message with argument into a specified
 * address which is passed in the argument. 
 */

#ifdef DCF_MEASURE_MAC_PROC_ENABLED
char name[10] = {'M','A','C','_','P','R','O','C','_','\0'};
#else

#ifdef DCF_ACK_REPLACE_DATA_CHECK_ENABLED
char name[12] = {'A','C','K','_','R','E','P','L','A','C','E','\0'};
#endif
char name[6] = {'N','a','m','e','_','\0'};
#endif
static bool board_data_cleared = false;
void print_text_at_insert_location(uint8_t* addr, const char* format, ...)
{
    char filename[50];  
    
    sprintf(filename, "%s%02x%02x%02x%02x%02x%02x.log", name,addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    if(!board_data_cleared){
        fclose(fopen(filename, "w"));
        board_data_cleared = true;
    }      
    va_list args;
    va_start (args, format);   
    FILE* stream;
    stream = fopen(filename,"a+");     
    vfprintf (stream, format, args);
    va_end (args); 
    fclose(stream);  
}
