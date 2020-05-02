#ifndef SIGNAL_H
#define SIGNAL_H
#include <QObject>
#include <QMetaType>
class Signal
{


public:
    Signal(){}
    ~Signal(){}
 //   Signal(const Signal &){};

    double startR;
    double startL;
    double encL;
    double encR;
    double tmpPencL;
    double tmpPencR;
    double robotFi;
    double distx;
    double disty;

};
Q_DECLARE_METATYPE(Signal)
#endif // SIGNAL_H
