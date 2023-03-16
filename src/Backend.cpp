#include "Backend.h"

Backend::Backend(QObject *parent)
    : QObject{parent}
{    
    timer = new QTimer(this);

    timer->start(100);

    connect(timer, &QTimer::timeout, this, [=](){
        setStatus(g_status);
    });
}


