#ifndef _TIEVLOG_H
#define _TIEVLOG_H

#include <string.h>

#include <iostream>

enum LOG_LEVEL { INFO, WARNING, ERROR, FATAL };

class TiEVLog {
 public:
  TiEVLog(LOG_LEVEL level) : log_level(level){};
  ~TiEVLog();
  static std::ostream &log(LOG_LEVEL level, const std::string &filename,
                           int line);

 private:
  static std::ostream &getStream(LOG_LEVEL level);
  LOG_LEVEL            log_level;
};

#define filename(path) strrchr(path, '/') ? strrchr(path, '/') + 1 : path
#define LOG(level)     TiEVLog(level).log(level, filename(__FILE__), __LINE__)

#endif
