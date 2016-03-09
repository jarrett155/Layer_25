#include <mac_log.h>
#ifdef MAC_LOGGING_ENABLED
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

static mac_log_level filter_level = MAC_LOG_LEVEL_VERBOSE;
static bool logfile_cleared = false;

/** this function is used with MARCO to write the desired message on the screen*/
void mac_log_write(mac_log_level level, const char *caller, const char *format, ...)
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
void mac_log_file_write(const char* filename,mac_log_level level, const char *caller, 
        const char *format, ...)
{
    if(!logfile_cleared){
        FILE* fp = fopen(filename, "w");
        if(fp != NULL){
            time_t rawtime;
            time ( &rawtime );
            fprintf(fp, "MAC Log File: [%s] \n", ctime (&rawtime));
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

#ifdef MAC_RX_CONTENT_LOGGING_ENABLED

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
static bool rx_content_logfile_cleared = false;
void rx_content_log_info(uint8_t* addr, const char* format, ...)
{
    char filename[50];   
    sprintf(filename, "RX_CONTENT_%02x%02x%02x%02x%02x%02x.log", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    if(!rx_content_logfile_cleared){
        fclose(fopen(filename, "w"));
        rx_content_logfile_cleared = true;
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
 * this function is used with rx_content_log_info, this function is used to 
 * print out every bit of a uint16_t type, the LSB is on the right side.
 * @param addr
 * @param bit_to_print
 */
void rx_content_print_bit(uint8_t *addr, uint16_t bit_to_print)
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
        rx_content_log_info(addr, "%d", (((mask<<i) & result)>>i) & mask);
    } 
}

#endif