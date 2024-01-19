#ifndef __COMETS_LOGGER_H__
#define __COMETS_LOGGER_H__

#ifdef __FILE_NAME__
// GCC 12 and up.
#define COMET_LOGGER_FILE_NAME __FILE_NAME__
#else
#define COMET_LOGGER_FILE_NAME __FILE__
#endif

#define COMET_LOG_IMPL(suffix, format, args) printf(COMET_LOGGER_FILE_NAME suffix ": " format "\n", args)

#define COMET_LOG(format, ...) COMET_LOG_IMPL("", format, __VA_ARGS__)
#define COMET_ERR(format, ...) COMET_LOG_IMPL(" ERROR" format, __VA_ARGS__)

#endif
