#ifndef SIGNAL_H
#define SIGNAL_H
#include <QObject>
#include <QMetaType>
class Signal
{


public:
    Signal(){};
    ~Signal(){};
 //   Signal(const Signal &){};

    double startR;
    double startL;
    double encL;
    double encR;
    double pEncL;
    double pEncR;
    double robotFi;

};
Q_DECLARE_METATYPE(Signal);
#endif // SIGNAL_H
