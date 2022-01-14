/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-25 21:54:13
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-10-25 21:54:13
 */


#include "sys_logger.h"
#include <ctime>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


#define ONE_DAY_SECOND 60*60*24
SysLogger* SysLogger::instance_ = NULL;

SysLogger* SysLogger::GetInstance()
{
    if (instance_ == NULL) {
        instance_ = new SysLogger();
    }
    
    return instance_;
}

SysLogger::SysLogger():log_fp_(nullptr)
{
    min_level_ = 0;

    log_file_head_ = new char[MAX_FILE_HEAD_LEN];
    memset(log_file_head_, 0, MAX_FILE_HEAD_LEN);

    log_file_name_ = new char[MAX_FILE_NAME_LEN];
    memset(log_file_name_, 0, MAX_FILE_NAME_LEN);

    log_buf_ = new char[LOG_BUF_SIZE];
    memset(log_buf_, 0, LOG_BUF_SIZE);

    mutex_ = new pthread_mutex_t;
    pthread_mutex_init(mutex_, NULL);
}


SysLogger::~SysLogger()
{
    if (log_file_head_ != NULL) {
        delete[] log_file_head_;
        log_file_head_ = NULL;
    }

    if(log_file_name_ != NULL) {
        delete[] log_file_name_;
        log_file_name_ = NULL;
    }
    
    if (log_buf_ != NULL) {
        delete[] log_buf_;
        log_buf_ = NULL;
    }
    
    if (log_fp_ != nullptr) {
        fclose(log_fp_);
        log_fp_ = nullptr;
    }
    
    pthread_mutex_destroy(mutex_);
    
    if (mutex_ != NULL) {
        delete mutex_;
    }
}

bool SysLogger::InitLogger(const char* file_name, int min_level,int save_day)
{
    strncpy(log_file_head_, file_name, MAX_FILE_HEAD_LEN - 1);
    log_file_head_[MAX_FILE_HEAD_LEN - 1] = '\0';

    snprintf(log_file_name_, MAX_FILE_NAME_LEN - 1, "%s.log",log_file_head_);
    log_file_name_[MAX_FILE_NAME_LEN - 1] = '\0';
    
    time_t now =time(NULL);
    localtime_r(&now, &sys_tm_);
    
    if(!create_day_file(&sys_tm_))
    {
        return false;
    }

    if (min_level >=0 && min_level <= 4) {
        min_level_ = min_level;
    }

    save_day_ = save_day;

    return true;
}

void SysLogger::WriteLog(int level, const char* exec_file, int exec_line, int tid, const char* format, ...)
{
    pthread_mutex_lock(mutex_);
    va_list valst;
    va_start(valst, format);
    set_log(level, exec_file, exec_line, tid, format, valst);
    va_end(valst);

    struct stat fstat;
    if(stat(log_file_name_, &fstat) == -1){

        create_day_file(NULL);
    }

    if (level < min_level_ || log_fp_ == nullptr) {
        pthread_mutex_unlock(mutex_);
        return;
    }

    fputs(log_buf_, log_fp_);
    fflush(log_fp_);
    
    pthread_mutex_unlock(mutex_);
}

// Used for mod exit
void SysLogger::WriteLogNoLock(int level, const char* exec_file, int exec_line, int tid, const char* format, ...)
{
    if (level < min_level_) {
        return;
    }
    
    va_list valst;
    va_start(valst, format);
    set_log(level, exec_file, exec_line, tid, format, valst);
    va_end(valst);
    fputs(log_buf_, log_fp_);
    fflush(log_fp_);
    
    // if (log_fp_ != NULL) {
    // fclose(log_fp_);
    // log_fp_ = NULL;
    // }
    
}

void SysLogger::set_log(int level, const char* exec_file, int exec_line, int tid, const char* format, va_list valst)
{
    char exec_filename[MAX_FILE_NAME_LEN];
    memset(exec_filename, 0, MAX_FILE_NAME_LEN);
    const char* pch = strrchr(exec_file, '/');
    
    if (pch == NULL) {
        strncpy(exec_filename, exec_file, MAX_FILE_NAME_LEN - 1);
    } else {
        strncpy(exec_filename, pch + 1, MAX_FILE_NAME_LEN - 1);
    }
    
    char levstr[16];
    memset(levstr, 0, 16);
    
    switch (level) {
        case LEVEL_SOCKET_DEBUG:
        case LEVEL_DEBUG:
            strcpy(levstr, "DEBUG");
            break;
        case LEVEL_INFO:
            strcpy(levstr, "INFO");
            break;
        case LEVEL_WARNING:
            strcpy(levstr, "WARN");
            break;
        case LEVEL_ERROR:
            strcpy(levstr, "ERROR");
            break;
        default:
            strcpy(levstr, "INFO");
            break;
    }
    
    memset(log_buf_, 0, LOG_BUF_SIZE);

    struct timeval now = {0, 0};
    gettimeofday(&now, NULL);
    struct tm sys_tm;
    localtime_r(&(now.tv_sec), &sys_tm);
    
    int n = snprintf(log_buf_, 128, "\n%04d-%02d-%02d %02d:%02d:%02d,%03d <%s> [%s:%d] [%d] ",
                     sys_tm.tm_year+1900, sys_tm.tm_mon+1, sys_tm.tm_mday, 
                     sys_tm.tm_hour, sys_tm.tm_min, sys_tm.tm_sec, now.tv_usec / 1000,
                     levstr, exec_filename, exec_line, tid);
    vsnprintf(log_buf_ + n, LOG_BUF_SIZE - n, format, valst);
#ifdef DEBUG
    printf("%s", log_buf_);
#endif

}

bool SysLogger::create_day_file(const struct tm *sys_tm)
{
    if(log_fp_ != nullptr) {
        fclose(log_fp_);
        log_fp_ = nullptr;
    }
    
    log_fp_ = fopen(log_file_name_, "a+");
    
    if (log_fp_ == nullptr) {
        return false;
    }

    return true;
}
bool SysLogger::remove_day_file(const struct tm *sys_tm)
{
    char old_file_name[MAX_FILE_NAME_LEN] ={0};

    sprintf(old_file_name,"%s_%d-%d-%d.log",log_file_head_,sys_tm->tm_year + 1900, sys_tm->tm_mon + 1, sys_tm->tm_mday);
    if(access(old_file_name,F_OK) == 0)
    {
        unlink(old_file_name);
    }

    return true;
}
