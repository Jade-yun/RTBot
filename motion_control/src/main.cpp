#include "EtherCATInterface.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>

#include "SharedMemoryManager.h"

int main() {
    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread;
    int ret; 

    signal(SIGINT, signal_handler);

    ret = init_ethercat(); //初始化Ethercat
    printf("master enable\n");
    if(ret) {
        printf("Failed to init EtherCAT master.\n");
        exit(-2);
    }

    if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        printf("mlockall() failed: %m\n");
        exit(-2);
    }

    ret = pthread_attr_init(&attr);
    if(ret) {
        printf("init pthread attributes failed\n");
        return ret;
    }

    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if(ret) {
        printf("pthread setschedpolicy failed\n");
        return ret;
    }

    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if(ret) {
        printf("pthread setschedparam failed\n");
        return ret;
    }

    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if(ret) {
        printf("pthread setinheritsched failed\n");
        return ret;
    }

    ret = pthread_create(&thread, &attr, simple_cyclic_task, NULL);
    if(ret) {
        printf("create pthread failed: %s\n", strerror(ret));
        return ret;
    }

    ret = pthread_join(thread, NULL);
    if(ret) {
        printf("join pthread failed: %m\n");
    }
}
