#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "shared_data.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_spinBoxPos_valueChanged(int arg1);

    void on_spinBoxVelocity_valueChanged(int arg1);

    void on_btnStart_clicked();

    void on_btnStop_clicked();

private:
    Ui::MainWindow *ui;
 
private:
    SharedData* data;
};
#endif // MAINWINDOW_H
