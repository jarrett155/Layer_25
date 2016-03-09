/* 
 * File:   dcf_log.h
 * Author: bx
 *
 * 
 * Created on February 22, 2015, 8:56 PM
 */

#ifndef DCF_LOG_H
#define	DCF_LOG_H
#include <stdio.h>
#include <stdint.h>

//#define DCF_LOGGING_ENABLED
//#define DCF_TX_CONTENT_LOGGING_ENABLED 
//#define DCF_MEASURE_MAC_PROC_ENABLED 
#define DISPLAY_DCF_TX_STATISTICS  


/**
 * Severity levels for logging functions
 */
typedef enum {
    DCF_LOG_LEVEL_VERBOSE,  /**< Verbose level logging */
    DCF_LOG_LEVEL_DEBUG,    /**< Debug level logging */
    DCF_LOG_LEVEL_INFO,     /**< Information level logging */
    DCF_LOG_LEVEL_WARNING,  /**< Warning level logging */
    DCF_LOG_LEVEL_ERROR,    /**< Error level logging */
    DCF_LOG_LEVEL_CRITICAL, /**< Fatal error level logging */
    DCF_LOG_LEVEL_SILENT    /**< No output */
} dcf_log_level;

/**
 * @defgroup LOG_MACROS Logging macros
 * @{
 */

/** Logs a verbose message. Does not include function/line information */
#define dcf_log_verbose(FILENAME,...) \
    DCF_LOG_FILE_WRITE(FILENAME,DCF_LOG_LEVEL_VERBOSE, "[VERBOSE", __VA_ARGS__)

/** Logs a debug message. Does not include function/line information */
#define dcf_log_debug(FILENAME,...) \
    DCF_LOG_FILE_WRITE(FILENAME,DCF_LOG_LEVEL_DEBUG, "[DEBUG", __VA_ARGS__)

/** Logs an info message. Does not include function/line information*/
#define dcf_log_info(FILENAME,...) \
    DCF_LOG_FILE_WRITE(FILENAME,DCF_LOG_LEVEL_INFO, "[INFO" , __VA_ARGS__)

/** Logs a warning message. Includes function/line information */
#define dcf_log_warning(...) \
    DCF_LOG_WRITE(DCF_LOG_LEVEL_WARNING, "[WARNING", __VA_ARGS__)

/** Logs an error message. Includes function/line information */
#define dcf_log_error(...) \
    DCF_LOG_WRITE(DCF_LOG_LEVEL_ERROR, "[ERROR", __VA_ARGS__)

/** Logs a critical error message. Includes function/line information */
#define dcf_log_critical(...) \
    DCF_LOG_WRITE(DCF_LOG_LEVEL_CRITICAL, "[CRITICAL", __VA_ARGS__)

#ifdef LOG_INCLUDE_FILE_INFO
#   define LOG_WRITE(LEVEL, LEVEL_STRING, ...) \
    do { log_write(LEVEL, LEVEL_STRING  \
                     " @ "  __FILE__ ":" _LOG_STRINGIFY_(__LINE__) "] " \
                     __VA_ARGS__); \
    } while (0)
#else
#   define DCF_LOG_WRITE(LEVEL, LEVEL_STRING, ...) \
    do { dcf_log_write(LEVEL, __func__, LEVEL_STRING "] " __VA_ARGS__); } while (0)           
#   define DCF_LOG_FILE_WRITE(FILENAME,LEVEL, LEVEL_STRING, ...) \
    do { dcf_log_file_write(FILENAME, LEVEL, __func__, LEVEL_STRING "] " __VA_ARGS__); } while (0)
#endif
 
//declaration
#ifdef DCF_LOGGING_ENABLED
void dcf_log_write(dcf_log_level level, const char *caller, const char *format, ...);//writes to stdout
void dcf_log_file_write(const char* filename,dcf_log_level level, const char *caller, 
        const char *format, ...); //writes to log files

#else
#define dcf_log_write(level, caller, format, ...)  do {} while(0)
#define dcf_log_file_write(filename, level, caller, format, ...)   do {} while(0)
#endif

//non-standard log file, for extra one piece data 
void print_text_at_insert_location(uint8_t* addr, const char* format, ...);  
//print out detailed transmission information
#ifdef DCF_TX_CONTENT_LOGGING_ENABLED
    void tx_content_log_info(uint8_t* addr, const char* format, ...);  
    void tx_content_print_bit(uint8_t *addr, uint16_t bit_to_print);
#endif

#endif	/* DCF_LOG_H */

