#include <iostream>
#include <unistd.h>
#include <thread>


// #include "ethercat/EtherCATInterface.h"
#include "Utilities/PeriodicTask.h"
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/circular_buffer.hpp>
#include <thread>

class EtherCATInterface
{
public:
    EtherCATInterface(){};
    ~EtherCATInterface(){};

    bool init(){
        cnt = 1;
        return true;
    }
    void runEtherCat(){
        // printf("hello %d\n", cnt++);

        for (int i = 0; i < 50000000; i++)
        {
            int j = i * cnt;
            int k = i^2 + j^2;
        }
    }
    void runEtherCat2(){
        // printf("hello %d\n", cnt++);

        for (int i = 0; i < 500000000; i++)
        {
            int j = i * cnt;
            int k = i^2 + j^2;
        }
    }

private:
    static uint64_t cnt;
};

uint64_t EtherCATInterface::cnt = 0;

int main()
{


    PeriodicTaskManager taskManager;

    EtherCATInterface ethercatIf;

    ethercatIf.init();
    PeriodicMemberFunction<EtherCATInterface> ecatTask(
        &taskManager, .001, "ecat", &EtherCATInterface::runEtherCat, &ethercatIf);

    ecatTask.start();

    ethercatIf.init();
    PeriodicMemberFunction<EtherCATInterface> ecatTask2(
        &taskManager, .01, "ecat2", &EtherCATInterface::runEtherCat2, &ethercatIf);

    ecatTask2.start();

    while (1)
    {
        taskManager.printStatus();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // signal(SIGINT, signal_handler);

    // while (1) 
    // {
    //     usleep(1000 * 1000 );
    //     taskManager.printStatus();
    //     // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
    // }

    // boost::circular_buffer<int> buffer(24);


    // std::thread producer([&buffer](){
    //     static int cnt = 0;
    //     while (1) 
    //     {
    //         buffer.push_back(cnt++);
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // });

    // std::thread consumer([&buffer](){
    //     while (1) 
    //     {
    //         if (!buffer.empty())
    //         {
    //             int value = buffer.front();
    //             buffer.pop_front();
    //             printf("value: %d\n", value);

    //         }
    //         std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //     }
    // });

    // producer.join();
    // consumer.join();

}
