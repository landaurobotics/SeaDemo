#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QQuickItem>

#include <QTimer>

class Backend : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int ctrlMode READ ctrlMode WRITE setCtrlMode NOTIFY ctrlModeChanged)
    Q_PROPERTY(bool enable READ enable WRITE setEnable NOTIFY enableChanged)
    Q_PROPERTY(int status READ status WRITE setStatus NOTIFY statusChanged)
    Q_PROPERTY(double impStiff READ impStiff WRITE setImpStiff NOTIFY impStiffChanged)
    Q_PROPERTY(double impDamp READ impDamp WRITE setImpDamp NOTIFY impDampChanged)

    QML_ELEMENT
public:
    explicit Backend(QObject *parent = nullptr);

    //!< ctrlMode
    double ctrlMode() { return ctrl_mode;}
    void setCtrlMode(double value) {
        ctrl_mode = value;
        emit ctrlModeChanged();
    }

    //!< enable
    double enable() { return enabled;}
    void setEnable(double value) {
        enabled = value;
        emit enableChanged();
    }

    //!< status
    double status() { return g_status;}
    void setStatus(double value) {
        g_status = value;
        emit statusChanged();
    }

    //!< impStiff
    double impStiff() { return imp_stiff;}
    void setImpStiff(double value) {

        imp_stiff = value;
        emit impStiffChanged();
    }

    //!< impDamp
    double impDamp() { return imp_damp;}
    void setImpDamp(double value) {
        imp_damp = value;
        emit impDampChanged();
    }


//    Q_INVOKABLE void startMotion();
//    Q_INVOKABLE void stopMotion();

signals:
    void ctrlModeChanged();
    void enableChanged();
    void statusChanged();
    void impStiffChanged();
    void impDampChanged();

private:
    QTimer* timer;

    double ctrl_mode;
    bool enabled;
    double g_status;
    double imp_stiff;
    double imp_damp;

};

#endif // BACKEND_H
