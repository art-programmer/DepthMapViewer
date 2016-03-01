#include "main_window.h"

#include <QHBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include <QKeyEvent>
#include <QInputDialog>

#include "configuration.h"


using namespace structured_indoor_modeling;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    QWidget *central_widget = new QWidget;

    //    QWidget window;
//    window.resize(1024, 720);
    // window.resize(1280, 720);
    // window.resize(1000, 600);

    string data_directory; // = argv[1];

    /*
    string data_directory;
    for (int i = 1; i < argc; ++i) {
      string word = argv[i];
      data_directory = data_directory + word;

      if (i != argc - 1)
        data_directory = data_directory + string(" ");
    }
    */

    string suffix("");
//    if (argc > 2)
//      suffix = argv[2];

    Configuration configuration;
    {
      configuration.data_directory = "../Results";
      configuration.scene_index = 10001;
    }

    MainWidget* main_widget_1 = new MainWidget(configuration, suffix);
    MainWidget* main_widget_2 = new MainWidget(configuration, suffix);
    main_widget_2->setRenderingMode('O');

//    QScrollArea *area_1 = new QScrollArea();
//    area_1->setWidget(main_widget_1);
//    area_1->setWidgetResizable(true);
//    area_1->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
//    area_1->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
//    area_1->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
//    area_1->setMinimumSize(50, 50);

    QGridLayout* layout = new QGridLayout();
    layout->addWidget(main_widget_1, 0, 0);
    layout->addWidget(main_widget_2, 0, 1);

    setCentralWidget(central_widget);

    central_widget->setLayout(layout);


    setStyleSheet("QMainWindow {background: 'black';}");
//    setWindowTitle("Depth Map Viewer");
    resize(1400, 500);

    connect(this, SIGNAL(toStartMovement()), main_widget_1, SLOT(startMovement()));
    connect(this, SIGNAL(toStartMovement()), main_widget_2, SLOT(startMovement()));
    connect(this, SIGNAL(toResetScene(int)), main_widget_1, SLOT(resetScene(int)));
    connect(this, SIGNAL(toResetScene(int)), main_widget_2, SLOT(resetScene(int)));
}

void MainWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_P) {
        toStartMovement();
//        main_widget_1->startMovement();
//        main_widget_2->startMovement();
    }

    if (e->key() == Qt::Key_S) {
        bool ok;
        int scene_index = QInputDialog::getInt(this, "Input scene index.", "scene index: ", 1, 1, 20000, 1, &ok);
        if (ok == true)
            toResetScene(scene_index);
    }
}
