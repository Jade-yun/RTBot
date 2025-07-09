#include <iostream>
#include <unistd.h>
#include <thread>


// #include "ethercat/EtherCATInterface.h"
#include "Utilities/PeriodicTask.h"
#include <readerwriterqueue.h>
#include <thread>
#include <cmath>
#include <random>
#include <mutex>

std::mutex mutex;
int index;



moodycamel::ReaderWriterQueue<double> queue(1024);

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
        // thread_local std::mt19937_64 rng(std::random_device{}());

        // 获取当前时间作为基准
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        double t = std::chrono::duration<double>(duration).count(); // 当前秒数
    
        double result = 0.0;
        const double fundamental_freq = 2.0 * M_PI; // 基频（Hz）
    
        // 合成前5次傅里叶级数项：A_n * sin(n * ω * t + φ)
        for (int n = 1; n <= 5; ++n) {
            double amplitude = 1.0 / n;                // 振幅随 n 衰减
            double frequency = n * fundamental_freq;   // 第 n 次谐波
            double phase = 0.0;                        // 相位偏移
    
            result += amplitude * std::sin(frequency * t + phase);
        }

        if (mutex.try_lock())
        {
            index++;

            mutex.unlock();
        }
    
        // 将结果放入队列中
        while (!queue.try_enqueue(result)) {
            std::this_thread::yield();
        }
    }
    void runEtherCat2(){
        // printf("hello %d\n", cnt++);

        int recv_value;
        while (queue.try_dequeue(recv_value))
        {
            double result = 0.0;
    
            for (int i = 0; i < 100000; ++i) {
                double a = recv_value;
                double b = recv_value;
        
                // 示例浮点运算：平方根 + 平方 + sin/cos 等
                result += std::sqrt(a) * std::sin(b) + std::pow(a, 2.0);
            }

            if (mutex.try_lock())
            {
                index++;
    
                mutex.unlock();
            }
    
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
        &taskManager, .005, "ecat2", &EtherCATInterface::runEtherCat2, &ethercatIf);

    ecatTask2.start();

    while (1)
    {
        taskManager.printStatus();
        // taskManager.printStatusOfSlowTasks();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 1));
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
