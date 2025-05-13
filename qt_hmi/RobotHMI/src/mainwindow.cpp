#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "Utilities/SharedMemoryManager.h"
#include "Parameters/SharedDataType.h"
#include <QTimer>
#include <QDebug>

SharedMemoryManager<SharedMemoryData> shm(SharedMemoryManager<SharedMemoryData>::Attacher);

int pos_times = 1;

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

    timer = new QTimer(this);
    timer->start(1*1000);

    connect(timer, &QTimer::timeout, this, [=](){

        HighLevelCommand cmd;
        cmd.command_type = HighLevelCommandType::MoveJ;
        for (int i = 0; i < NUM_JOINTS; i++)
        {
           cmd.movej_params.target_joint_pos[i] = i * pos_times;
        }
        pos_times++;

        if (shm().cmd_queue.push(cmd))
        {

        }
    });
//    shm().cmd_queue.push()
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
//    data->command.mode = ControlMode::MOVE_JOINT;
       timer->start(1 * 1000);

}

void MainWindow::on_btnStop_clicked()
{
//    data->command.mode = ControlMode::STOP;
       timer->stop();

}
