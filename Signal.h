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

    double fintargx;
    double fintargy;
    double walldet;
    double wallfolow;
    double navstate;
    double startstate;
    double robotFi;
    double distx;
    double disty;
    double firstime;
    double go2finTarg;

};
Q_DECLARE_METATYPE(Signal)
#endif // SIGNAL_H
