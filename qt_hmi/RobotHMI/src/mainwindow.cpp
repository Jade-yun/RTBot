#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "shared_data.h"

void updateCommand(int runFlag) {
    SharedMemoryManager& manager = SharedMemoryManager::getInstance();
    SharedData* data = manager.getData();

    if (data) {
        pthread_mutex_lock(&data->mutex);
        data->command.run_flag = runFlag;
        pthread_mutex_unlock(&data->mutex);
    }
}

void displayStatus() {
    SharedMemoryManager& manager = SharedMemoryManager::getInstance();
    SharedData* data = manager.getData();

    if (data) {
        pthread_mutex_lock(&data->mutex);
        std::cout << "Position: " << data->status.position << std::endl;
        std::cout << "Velocity: " << data->status.velocity << std::endl;
        pthread_mutex_unlock(&data->mutex);
    }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QWidget* central = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(central);

    posLabel = new QLabel("Position: 0.0", this);
    layout->addWidget(posLabel);

    startBtn = new QPushButton("Start", this);
    stopBtn = new QPushButton("Stop", this);
    layout->addWidget(startBtn);
    layout->addWidget(stopBtn);

    setCentralWidget(central);

    // 连接共享内存
    shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
    data = static_cast<SharedData*>(mmap(nullptr, sizeof(SharedData),
                PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    connect(startBtn, &QPushButton::clicked, this, [=]() {
        pthread_mutex_lock(&data->mutex);
        data->command.run_flag = 1;
        pthread_mutex_unlock(&data->mutex);
    });

    connect(stopBtn, &QPushButton::clicked, this, [=]() {
        pthread_mutex_lock(&data->mutex);
        data->command.run_flag = 0;
        pthread_mutex_unlock(&data->mutex);
    });

    // 定时读取控制状态
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateStatus);
    timer->start(100);
}

~MainWindow() 
{
    delete ui;
    munmap(data, sizeof(SharedData));
    close(shm_fd);
}

