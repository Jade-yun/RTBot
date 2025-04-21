#pragma once

#include <QObject>
#include <QTimer>
#include "SharedMemoryManager.h"

class HmiController : public QObject {
    Q_OBJECT
public:
    explicit HmiController(QObject* parent = nullptr);
    void start();

signals:
    void stateUpdated();  // 可以连接 UI，用于刷新关节状态等

private slots:
    void onUpdate();

private:
    QTimer timer_;
    SharedData* data_;
};
