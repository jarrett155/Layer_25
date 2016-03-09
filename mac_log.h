/* 
 * File:   mac_log.h
 * Author: bx
 *
 * Created on March 6, 2015, 6:25 PM
 */

#ifndef MAC_LOG_H
#define	MAC_LOG_H

//#define MAC_LOGGING_ENABLED
//#define MAC_RX_CONTENT_LOGGING_ENABLED
#define DISPLAY_MAC_FRAME_STATISTICS

#include <stdint.h>
/**
 * Severity levels for logging functions
 */
typedef enum {
    MAC_LOG_LEVEL_VERBOSE,  /**< Verbose level logging */
    MAC_LOG_LEVEL_DEBUG,    /**< Debug level logging */
    MAC_LOG_LEVEL_INFO,     /**< Information level logging */
    MAC_LOG_LEVEL_WARNING,  /**< Warning level logging */
    MAC_LOG_LEVEL_ERROR,    /**< Error level logging */
    MAC_LOG_LEVEL_CRITICAL, /**< Fatal error level logging */
    MAC_LOG_LEVEL_SILENT    /**< No output */
} mac_log_level;

/**
 * @defgroup LOG_MACROS Logging macros
 * @{
 */

/** Logs a verbose message. Does not include function/line information */
#define mac_log_verbose(FILENAME,...) \
    MAC_LOG_FILE_WRITE(FILENAME,MAC_LOG_LEVEL_VERBOSE, "[VERBOSE", __VA_ARGS__)

/** Logs a debug message. Does not include function/line information */
#define mac_log_debug(FILENAME,...) \
    MAC_LOG_FILE_WRITE(FILENAME,MAC_LOG_LEVEL_DEBUG, "[DEBUG", __VA_ARGS__)

/** Logs an info message. Does not include function/line information*/
#define mac_log_info(FILENAME,...) \
    MAC_LOG_FILE_WRITE(FILENAME,MAC_LOG_LEVEL_INFO, "[INFO" , __VA_ARGS__)

/** Logs a warning message. Includes function/line information */
#define mac_log_warning(...) \
    MAC_LOG_WRITE(MAC_LOG_LEVEL_WARNING, "[WARNING", __VA_ARGS__)

/** Logs an error message. Includes function/line information */
#define mac_log_error(...) \
    MAC_LOG_WRITE(MAC_LOG_LEVEL_ERROR, "[ERROR", __VA_ARGS__)

/** Logs a critical error message. Includes function/line information */
#define mac_log_critical(...) \
    MAC_LOG_WRITE(MAC_LOG_LEVEL_CRITICAL, "[CRITICAL", __VA_ARGS__)

#ifdef LOG_INCLUDE_FILE_INFO
#   define LOG_WRITE(LEVEL, LEVEL_STRING, ...) \
    do { log_write(LEVEL, LEVEL_STRING  \
                     " @ "  __FILE__ ":" _LOG_STRINGIFY_(__LINE__) "] " \
                     __VA_ARGS__); \
    } while (0)
#else
#   define MAC_LOG_WRITE(LEVEL, LEVEL_STRING, ...) \
    do { mac_log_write(LEVEL, __func__, LEVEL_STRING "] " __VA_ARGS__); } while (0)           
#   define MAC_LOG_FILE_WRITE(FILENAME,LEVEL, LEVEL_STRING, ...) \
    do { mac_log_file_write(FILENAME, LEVEL, __func__, LEVEL_STRING "] " __VA_ARGS__); } while (0)
#endif
 
//declaration
#ifdef MAC_LOGGING_ENABLED
void mac_log_write(mac_log_level level, const char *caller, const char *format, ...);//writes to stdout
void mac_log_file_write(const char* filename,mac_log_level level, const char *caller, 
        const char *format, ...); //writes to log files

#else
#define mac_log_write(level, caller, format, ...)  do {} while(0)
#define mac_log_file_write(filename, level, caller, format, ...)   do {} while(0)
#endif            

#ifdef MAC_RX_CONTENT_LOGGING_ENABLED
void rx_content_log_info(uint8_t* addr, const char* format, ...);  
void rx_content_print_bit(uint8_t *addr, uint16_t bit_to_print);
#endif

void print_text_at_insert_location(uint8_t* addr, const char* format, ...); 

#endif	/* MAC_LOG_H */

