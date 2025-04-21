#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "SharedMemoryManager.h"

#include <QTimer>
#include <QDebug>

//void updateCommand(int runFlag) {
//    SharedMemoryManager& manager = SharedMemoryManager::getInstance();
//    SharedData* data = manager.getData();

//    if (data) {
//        pthread_mutex_lock(&data->mutex);
//        data->command.run_flag = runFlag;
//        pthread_mutex_unlock(&data->mutex);
//    }
//}

//void displayStatus() {
//    SharedMemoryManager& manager = SharedMemoryManager::getInstance();
//    SharedData* data = manager.getData();

//    if (data) {
//        pthread_mutex_lock(&data->mutex);
//        std::cout << "Position: " << data->status.position << std::endl;
//        std::cout << "Velocity: " << data->status.velocity << std::endl;
//        pthread_mutex_unlock(&data->mutex);
//    }
//}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    SharedMemoryManager& shmManager = SharedMemoryManager::getInstance();
    if (!shmManager.init(ShmMode::Attacher)) {
        qWarning() << "Failed to attach to shared memory.";
    }
    // 读取共享内存数据，如刷新 UI
    data = shmManager.getData();

    QTimer* timer = new QTimer(this);
    timer->start(1 * 1000);

    connect(timer, &QTimer::timeout, this, [=](){
        qDebug() << data->state.heartbeat_counter;
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_spinBoxPos_valueChanged(int arg1)
{

}

void MainWindow::on_spinBoxVelocity_valueChanged(int arg1)
{

}

void MainWindow::on_btnStart_clicked()
{
    data->command.mode = ControlMode::MOVE_JOINT;
}

void MainWindow::on_btnStop_clicked()
{
    data->command.mode = ControlMode::STOP;
}
