#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>

#ifndef QT_NO_OPENGL
#include "main_widget.h"
#endif


class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);

private:
    structured_indoor_modeling::MainWidget* main_widget_1;
    structured_indoor_modeling::MainWidget* main_widget_2;

protected:
    void keyPressEvent(QKeyEvent *e);

signals:
    void toStartMovement();
    void toResetScene(const int scene_index);

public slots:

};

#endif // MAIN_WINDOW_H
