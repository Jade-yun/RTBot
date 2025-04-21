#include "HmiController.h"
#include <QDebug>

HmiController::HmiController(QObject* parent)
    : QObject(parent), data_(nullptr)
{
    data_ = SharedMemoryManager::getInstance().getData();
    connect(&timer_, &QTimer::timeout, this, &HmiController::onUpdate);
    timer_.setInterval(100);  // 每 100ms 调用一次
}

void HmiController::start() {
    if (!data_) {
        qCritical() << "[HMI] Shared memory not initialized!";
        return;
    }

    timer_.start();
    qDebug() << "[HMI] Heartbeat & command update started.";
}

void HmiController::onUpdate() {
    if (!data_) return;

    // ⏱️ 更新心跳计数器
    data_->state.heartbeat_counter.fetch_add(1);

    // ✅ 如果 new_cmd 标志为 true，表示 HMI 写入了新指令
    if (data_->command.new_cmd.exchange(false)) {
        qDebug() << "[HMI] 下发控制命令: mode = " << static_cast<int>(data_->command.mode);
        // 也可触发日志/动画/提示等
    }

    // ✅ UI 可选更新：发出信号
    emit stateUpdated();
}
