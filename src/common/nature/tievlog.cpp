#include "tievlog.h"

std::ostream &TiEVLog::log(LOG_LEVEL level, const std::string &filename,
                           int line) {
  return getStream(level) << "[" << filename << ":" << line << "] "
                          << std::flush;
}

std::ostream &TiEVLog::getStream(LOG_LEVEL level) {
  return INFO == level ? std::cout
                       : (FATAL == level ? std::cerr << "\033[1;31m"
                                         : std::cerr << "\033[1;33m");
}

TiEVLog::~TiEVLog() {
  getStream(log_level) << "\033[0m" << std::endl << std::flush;
  if (log_level == FATAL) abort();
}