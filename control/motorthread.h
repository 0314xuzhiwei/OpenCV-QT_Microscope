#ifndef RETMOTORTHREAD_H
#define RETMOTORTHREAD_H

#include <QThread>
#include <./model/motorctr.h>


class MotorThread : public QThread
{
public:
    MotorThread();
    void Stop();

protected:
    void run();

private:
    MotorCtr* motor;
    int stopFlage;
};

#endif // RETMOTORTHREAD_H
