#include "mainwindow.h"
#include <QApplication>
#include <QMetaType>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType<Signal>();
    MainWindow w;
    w.show();

    return a.exec();
}
