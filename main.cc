#include <QApplication>
#include <QLabel>
#include <QDir>

#include <iostream>
#include <fstream>
#include <string>

#include "main_window.h"


using namespace std;

int main(int argc, char *argv[]) {
//  if (argc < 2) {
//    cerr << "Usage: " << argv[0] << " data_directory [suffix=_other]" << endl;
//    exit (1);
//  }
  QApplication app(argc, argv);

  QDir bin(QCoreApplication::applicationDirPath());

  /* Set working directory */

//#if defined(Q_WS_MAC)
  bin.cdUp();    /* Fix this on Mac because of the .app folder, */
  bin.cdUp();    /* which means that the actual executable is   */
  bin.cdUp();    /* three levels deep. Grrr.                    */
//#endif
  QDir::setCurrent(bin.absolutePath());

  app.setApplicationName("viewer");

#ifndef QT_NO_OPENGL
  MainWindow window;
  window.show();

#else
  QLabel note("OpenGL Support required");
  note.show();
#endif
  return app.exec();
}
