#include <iostream>

#include <QOpenGLShaderProgram>
#include "depth_map_renderer.h"
#include "main_widget.h"
#include <QGLFunctions>

#ifdef __linux__
#include <GL/glu.h>
#elif _WIN32
#include <windows.h>
#include <GL/glu.h>
#else
#include <GLUT/glut.h>
#endif

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {
  
  void MainWidget::RenderFloorplan(const double alpha,
                   const bool emphasize,
                   const double height_adjustment) {
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);

    // glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE);
    // glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE_MINUS_SRC_ALPHA);
    // glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ZERO);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);


    glDisable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);

    glPopAttrib();
  }

  void MainWidget::RenderPanorama(const double alpha) {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    
    glEnable(GL_TEXTURE_2D);
//    glEnable(GL_CULL_FACE);
    glDisable(GL_CULL_FACE);

//    glClearColor(1, 1, 1, 1);

    depth_map_renderer.Render(alpha, &depth_map_program);

//    switch (navigation.GetCameraStatus()) {
//    case kPanorama: {
//      //panorama_renderers[navigation.GetCameraPanorama().start_index].Render(alpha, &panorama_program);
//      depth_map_renderer.Render(alpha, &panorama_program);
//      break;
//    }
//    default: {
//    }
//    }
    
    glDisable(GL_TEXTURE_2D);
    glPopAttrib();
  }
  
  
  // divide_by_alpha_mode
  // 0: Do not divide by alpha.
  // 1: Divide by alpha.
  // 2: Divide by alpha and overwrite the first.
  // 3: Divide by alpha and overwrite the second.
  void MainWidget::BlendFrames(const double weight, const int divide_by_alpha_mode) {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glEnable(GL_TEXTURE_2D);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, width(), 0, height());
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    //////////////////////////////////////////////////////////////////////
    if (!blend_program.bind()) {
      cerr << "Cannot bind." << endl;
      exit (1);
    }
    
    blend_program.setUniformValue("weight", static_cast<float>(weight));
    blend_program.setUniformValue("divide_by_alpha", divide_by_alpha_mode);
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texids[0]);
    glEnable(GL_TEXTURE_2D);
    
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texids[1]);
    glEnable(GL_TEXTURE_2D);
    
    glDisable(GL_DEPTH_TEST);
    
    blend_program.setUniformValue("tex0", 0);
    blend_program.setUniformValue("tex1", 1);
    
    glBegin(GL_QUADS);
    glDisable(GL_DEPTH_TEST);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    
    glTexCoord2f(1.0f, 0.0f);
    glVertex3f(width(), 0.0f, 0.0f);
    
    glTexCoord2f(1.0f, 1.0f);
    glVertex3f(width(), height(), 0.0f);
    
    glTexCoord2f(0.0f, 1.0f);
    glVertex3f(0, height(), 0.0f);
    glEnd();
    
    blend_program.release();
    
    glActiveTexture(GL_TEXTURE1);
    glDisable(GL_TEXTURE_2D);
    
    glActiveTexture(GL_TEXTURE0);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);
    
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    
    glPopAttrib();
  }
  

  void MainWidget::ClearDisplay() {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  void MainWidget::ClearDisplayWithWhite() {
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(background[0], background[1], background[2], 0);
    glClearColor(0.0, 0.0, 0.0, 0.0);
  }

}  // namespace structured_indoor_modeling
