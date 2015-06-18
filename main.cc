#include <QApplication>
#include <QHBoxLayout>
#include <QLabel>

#include <iostream>
#include <fstream>
#include <string>

#include "configuration.h"

#ifndef QT_NO_OPENGL
#include "main_widget.h"
#endif

using namespace std;
using namespace structured_indoor_modeling;

int main(int argc, char *argv[]) {
//  if (argc < 2) {
//    cerr << "Usage: " << argv[0] << " data_directory [suffix=_other]" << endl;
//    exit (1);
//  }
  QApplication app(argc, argv);
  app.setApplicationName("viewer");

#ifndef QT_NO_OPENGL
  QWidget window;
  window.resize(1024, 720);
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
  if (argc > 2)
    suffix = argv[2];

  Configuration configuration;
  {
    configuration.data_directory = "Data";
    configuration.num_layers = 3;
    configuration.scene_index = 1;
  }

  MainWidget* main_widget = new MainWidget(configuration, suffix);

  QHBoxLayout* layout = new QHBoxLayout();
  layout->addWidget(main_widget);
  window.setLayout(layout);
  window.show();

#else
  QLabel note("OpenGL Support required");
  note.show();
#endif
  return app.exec();
}
