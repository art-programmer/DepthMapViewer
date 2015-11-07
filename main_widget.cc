#include "main_widget.h"
#include "panorama.h"

#include <fstream>
#include <iostream>
#include <locale.h>
#include <math.h>
#include <Eigen/Dense>
#include <QMouseEvent>
#include <QDialog>
#include <QInputDialog>

#ifdef __linux__
#include <GL/glu.h>
#elif _WIN32
#include <windows.h>
#include <GL/glu.h>
//#ifndef __glew_h__
#include <GL/glew.h>
//#include <GL/glext.h>
//#endif
#else
#include <OpenGL/glu.h>
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "main_widget_util.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const int kNumBuffers = 2;

const double MainWidget::kRenderMargin = 0.2;
const double MainWidget::kFadeInSeconds = 0.2;
const double MainWidget::kFadeOutSeconds = 1.5;
// The background color is not exactly 0, because we want to treat a
// pure black pixel in panorama images as holes. Fragment shader
// handles the pure black pixel in a special way, and we want the
// background color to be intensity 1 (avoid pure black).
const Eigen::Vector3f MainWidget::kBackgroundBlack = Eigen::Vector3f(0.005, 0.005, 0.005);
const Eigen::Vector3f MainWidget::kBackgroundWhite = Eigen::Vector3f(1.0f, 1.0f, 1.0f);

MainWidget::MainWidget(const Configuration& configuration_tmp, const std::string& suffix, QWidget *parent) :
  QGLWidget(parent),
  file_io(configuration_tmp.data_directory),
  view_parameters(),
//  navigation(view_parameters),
  configuration(configuration_tmp)
{
  // Renderer initialization.
  {
    InitPanoramasPanoramaRenderers();
  }

  setFocusPolicy(Qt::ClickFocus);
  setMouseTracking(true);

  {
//    navigation.Init();
    const double kDefaultAspectRatio = 1.333;
    view_parameters.Init(kDefaultAspectRatio);
  }

//  SetPanoramaToRoom(floorplan, panorama_renderers, &panorama_to_room);
//  SetRoomToPanorama(floorplan, panorama_renderers, &room_to_panorama);
//  SetPanoramaDistanceTable(panorama_renderers, &panorama_distance_table);
  
  current_width = current_height = -1;

  prev_animation_linear = 0.0;
  prev_animation_trapezoid = 0.0;
  
  fresh_screen_for_panorama = true;
  fresh_screen_for_air = true;
  fresh_screen_for_floorplan = true;

  simple_click_time.start();
  double_click_time.start();
  object_animation_time.start();
  simple_click_time_offset_by_move = 0.0;
  mouse_down = false;

  polygon_or_indoor_polygon = false;

  render_backface = true;

  background = kBackgroundBlack;


  viewing_angle_y = 60;

  transition_progress = 1.0;
  transition_direction = 'N';
}

MainWidget::~MainWidget() {
  FreeResources();
}

void MainWidget::AllocateResources() {
  glGenTextures(kNumBuffers, texids);
  glGenFramebuffers(kNumBuffers, frameids);
  glGenRenderbuffers(kNumBuffers, renderids);
    
  for (int i = 0; i < kNumBuffers; ++i) {
    glBindTexture(GL_TEXTURE_2D, texids[i]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width(), height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);    
      
    glBindRenderbuffer(GL_RENDERBUFFER, renderids[i]);
    // glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width(), height());
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width(), height());
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
      
    glBindFramebuffer(GL_FRAMEBUFFER, frameids[i]);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texids[i], 0);
    // glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, renderids[i]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, renderids[i]);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER); 
    if (status != GL_FRAMEBUFFER_COMPLETE) {
      cerr << "not complete" << endl;
      exit (1);
    }
  }
}

void MainWidget::FreeResources() {
  if (current_width != -1) {
    glDeleteTextures(kNumBuffers, texids);
    glDeleteFramebuffers(kNumBuffers, frameids);
    glDeleteRenderbuffers(kNumBuffers, renderids);
  }
}

void MainWidget::InitPanoramasPanoramaRenderers() {
//  const int kMaxPanoramaId = 100;
//  vector<int> panorama_ids;
//  for (int p = 0; p < kMaxPanoramaId; ++p) {
//    ifstream ifstr;
//    ifstr.open(file_io.GetPanoramaImage(p).c_str());

//    if (!ifstr.is_open())
//      continue;
//    panorama_ids.push_back(p);
//  }
//  if (panorama_ids.empty()) {
//    cerr << "No panorama." << endl;
//    exit (1);
//  }

//  panoramas.resize(panorama_ids.size());
//  panorama_renderers.resize(panorama_ids.size());
//  for (int i = 0; i < (int)panorama_ids.size(); ++i) {
//    panoramas[i].Init(file_io, panorama_ids[i]);
//    panorama_renderers[i].Init(file_io, panorama_ids[i], &panoramas[i], this);
//  }


//    depth_maps.assign(configuration.num_layers, Panorama());
//    for (int layer_index = 0; layer_index < configuration.num_layers; layer_index++)
//    {
//        depth_maps[layer_index].Init(file_io, configuration.scene_index, layer_index);
//    }

    depth_map_renderer = DepthMapRenderer();

    depth_map_renderer.Init(file_io, configuration.scene_index, this);
//    image_width = depth_maps[0].Width();
//    image_height = depth_maps[0].Height();
}
  
void MainWidget::InitializeShaders() {
  // Override system locale until shaders are compiled
  setlocale(LC_NUMERIC, "C");

  // Compile vertex shader
  if (!blend_program.addShaderFromSourceFile(QOpenGLShader::Vertex, "blend_vshader.glsl"))
    close();
  
  // Compile fragment shader
  if (!blend_program.addShaderFromSourceFile(QOpenGLShader::Fragment, "blend_fshader.glsl"))
    close();
  
  // Link shader pipeline
  if (!blend_program.link())
    close();

  // Bind shader pipeline for use
  // if (!blend_program.bind())
  // close();

  // Compile vertex shader
  if (!depth_map_program.addShaderFromSourceFile(QOpenGLShader::Vertex, "depth_map_vshader.glsl"))
    close();
  // Compile fragment shader
  if (!depth_map_program.addShaderFromSourceFile(QOpenGLShader::Fragment, "depth_map_fshader.glsl"))
    close();
  // Link shader pipeline
  if (!depth_map_program.link())
    close();

//  panorama_program.setUniformValue("image_width", static_cast<float>(image_width));
//  panorama_program.setUniformValue("image_height", static_cast<float>(image_height));

  // Restore system locale
  setlocale(LC_ALL, "");
}

void MainWidget::initializeGL() {
  initializeOpenGLFunctions();

  InitializeShaders();
  glClearColor(background[0], background[1], background[2], 0);
  glClearColor(1.0, 1.0, 1.0, 0.0);

  glEnable(GL_DEPTH_TEST);
//  glEnable(GL_CULL_FACE);
  glDisable(GL_CULL_FACE);

  depth_map_renderer.InitGL(&depth_map_program);

  // Use QBasicTimer because its faster than QTimer
  timer.start(1000 / 60, this);
}

void MainWidget::resizeGL(int w, int h) {
//  glViewport(w / 2 - h * image_width / image_height / 2, 0, h * image_width / image_height, h);

//    int window_size = min(w, h);
//    glViewport(w / 2 - window_size / 2, h / 2 - window_size / 2, window_size, window_size);

    glViewport(0, 0, w, h);

  if (w != current_width || h != current_height) {
    FreeResources();
    AllocateResources();
  }  
}

void MainWidget::SetMatrices() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
//  cout << depth_maps[configuration.num_layers - 1].GetAverageDistance() * 5 << '\t' << view_parameters.GetFloorplanHeight() * 2.0 << endl;
//  exit(1);
//  const double max_distance = 0; //depth_maps[configuration.num_layers - 1].GetMaxDepth() * 500;  //view_parameters.GetFloorplanHeight() * 2.0;
//  const double min_distance = max_distance / 10000.0;

  const double max_distance = 10;
  const double min_distance = 0.0001;
  const double x_angle = 120 * M_PI / 180.0;
  const double y_angle = 2.0 * atan(tan(x_angle / 2.0) * height() / width());

  gluPerspective(viewing_angle_y, width() / static_cast<double>(height()), min_distance, max_distance);
//  gluPerspective(viewing_angle_y, image_width / static_cast<double>(image_height), min_distance, max_distance);

//  glFrustum(-1, 1, -1, 1, 0.01, 10);

//  glOrtho(-1, 1, -1, 1, 0.01, 10);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const Vector3d center(0, 0, 0);
  const Vector3d direction(0, 0, -1);
  if (direction.norm() == 0.0) {
    cerr << "ZERO" << endl;
    exit (1);
  }

  if (std::isnan(center[0]) || std::isnan(center[1]) || std::isnan(center[2]) ||
      std::isnan(direction[0]) || std::isnan(direction[1]) || std::isnan(direction[2]))
    cerr << "black?" << endl
         << center << endl
         << direction << endl;

  const Vector3d z_axis(0, 1, 0); // = view_parameters.GetZAxis();
  gluLookAt(center[0], center[1], center[2],
            center[0] + direction[0], center[1] + direction[1], center[2] + direction[2],
            z_axis[0], z_axis[1], z_axis[2]);

  GLint viewport_old[4];
  GLdouble modelview_old[16];
  GLdouble projection_old[16];
  for (int i = 0; i < 4; ++i)
    viewport_old[i] = viewport[i];
  for (int i = 0; i < 16; ++i) {
    modelview_old[i] = modelview[i];
    projection_old[i] = projection[i];
  }

  glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
  glGetDoublev( GL_PROJECTION_MATRIX, projection );
  glGetIntegerv( GL_VIEWPORT, viewport );

  for (int i = 0; i < 4; ++i) {
    if (viewport_old[i] != viewport[i]) {
      fresh_screen_for_panorama = true;
      fresh_screen_for_air = true;
      fresh_screen_for_floorplan = true;
      break;
    }
  }
  for (int i = 0; i < 16; ++i) {
    if (modelview_old[i] != modelview[i]) {
      fresh_screen_for_panorama = true;
      fresh_screen_for_air = true;
      fresh_screen_for_floorplan = true;
      break;
    }
  }
  for (int i = 0; i < 16; ++i) {
    if (projection_old[i] != projection[i]) {
      fresh_screen_for_panorama = true;
      fresh_screen_for_air = true;
      fresh_screen_for_floorplan = true;
      break;
    }
  }

//  if (prev_animation_linear != AnimationLinear()) {
//    fresh_screen_for_panorama = true;
//    fresh_screen_for_air = true;
//    fresh_screen_for_floorplan = true;
//    prev_animation_linear = AnimationLinear();
//  }

//  if (prev_animation_trapezoid != AnimationTrapezoid()) {
//    fresh_screen_for_panorama = true;
//    fresh_screen_for_air = true;
//    fresh_screen_for_floorplan = true;
//    prev_animation_trapezoid = AnimationTrapezoid();
//  }
}

void MainWidget::PaintPanorama() {
    RenderPanorama(1.0);
}

  
void MainWidget::paintGL() {
  ClearDisplay();
    
  SetMatrices();
//  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  PaintPanorama();
}

  
//----------------------------------------------------------------------
// GUI
//----------------------------------------------------------------------
void MainWidget::mousePressEvent(QMouseEvent *e) {
}

void MainWidget::mouseReleaseEvent(QMouseEvent *e) {
}

void MainWidget::mouseDoubleClickEvent(QMouseEvent *e) {
  double_click_time.start();
  
//  mouse_down = true;
//  switch (navigation.GetCameraStatus()) {
//  case kPanorama: {
//    Vector3d direction = navigation.GetDirection();
//    direction[2] = 0.0;
//    direction.normalize();
//    Vector3d up(0, 0, 1);
//    Vector3d x_axis = direction.cross(up);
    
//    const double x_position = 2.0 * (e->localPos().x() / (double)viewport[2] - 0.5);
    
//    Vector3d new_direction =
//      direction + x_axis * x_position * tan(navigation.GetFieldOfViewInDegrees() / 2.0 * M_PI / 180.0);
//    navigation.MovePanorama(new_direction);
//    break;
//  }
//  default: {
//  }
//  }
}

void MainWidget::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Up) {
        transition_progress = 0.0;
        transition_direction = 'U';
    }
    if (e->key() == Qt::Key_Down) {
        transition_progress = 0.0;
        transition_direction = 'D';
    }
    if (e->key() == Qt::Key_Left) {
        transition_progress = 0.0;
        transition_direction = 'L';
    }
    if (e->key() == Qt::Key_Right) {
        transition_progress = 0.0;
        transition_direction = 'R';
    }
    if (e->key() == Qt::Key_I) {
        transition_progress = 0.0;
        transition_direction = 'F';
    }
    if (e->key() == Qt::Key_O) {
        transition_progress = 0.0;
        transition_direction = 'B';
    }
    if (e->key() == Qt::Key_P) {
        transition_progress = 0.0;
        transition_direction = 'P';
    }
    if (e->key() == Qt::Key_R) {
        depth_map_renderer.returnToOriginalViewPoint();
    }
    if (e->key() == Qt::Key_S) {
        bool ok;
        int scene_index = QInputDialog::getInt(this, "Input scene index.", "scene index: ", 1, 1, 20000, 1, &ok);
        if (ok == true)
            depth_map_renderer.resetScene(file_io, scene_index);
    }

    if (e->key() == Qt::Key_Space) {
        depth_map_renderer.TurnVBOOnOff();
    }
    if (e->key() == Qt::Key_A) {
        depth_map_renderer.setRenderingMode('A');
    }
    if (e->key() == Qt::Key_Q) {
        depth_map_renderer.setRenderingMode('O');
    }
    if (e->key() == Qt::Key_1) {
        depth_map_renderer.setRenderingMode('0');
    }
    if (e->key() == Qt::Key_2) {
        depth_map_renderer.setRenderingMode('1');
    }
    if (e->key() == Qt::Key_3) {
        depth_map_renderer.setRenderingMode('2');
    }
    if (e->key() == Qt::Key_4) {
        depth_map_renderer.setRenderingMode('3');
    }
    if (e->key() == Qt::Key_5) {
        depth_map_renderer.setRenderingMode('4');
    }
    if (e->key() == Qt::Key_6) {
        depth_map_renderer.setRenderingMode('5');
    }
    if (e->key() == Qt::Key_7) {
        depth_map_renderer.setRenderingMode('6');
    }
    if (e->key() == Qt::Key_8) {
        depth_map_renderer.setRenderingMode('7');
    }
    if (e->key() == Qt::Key_9) {
        depth_map_renderer.setRenderingMode('8');
    }
    if (e->key() == Qt::Key_0) {
        depth_map_renderer.setRenderingMode('9');
    }

    updateGL();


//    if (e->key() == Qt::Key_Up)
//        depth_map_renderer.moveByZ(0.02);
//    if (e->key() == Qt::Key_Down)
//        depth_map_renderer.moveByZ(-0.02);
//    if (e->key() == Qt::Key_Left)
//        depth_map_renderer.moveByX(0.02);
//    if (e->key() == Qt::Key_Right)
//        depth_map_renderer.moveByX(-0.02);

//  const double kRotationAngle = 45.0 * M_PI / 180.0;
//  //----------------------------------------------------------------------
//  // Arrows.
//  if (e->key() == Qt::Key_H) {
//    cerr << "There are 4 different viewing modes." << endl
//         << "  A: Switch to panorama mode." << endl
//         << "  S: Switch to aerial mode." << endl
//         << "  D: Switch to floorplan mode." << endl
//         << "  F: Switch to tree-view mode." << endl
//         << endl
//         << "R: Change the rendering style for the back-facing walls." << endl
//         << "T: Toggle on/off rendering the back side of the wall." << endl
//         << "U: Change the background color between black and white." << endl
//         << "I: Save a screenshot." << endl
//         << "O: Toggle on/off rendering the object points." << endl
//         << "P: Change the mesh in the aerial mode (floorplan.txt or floorplan_detailed.txt)." << endl;
//  } else if (e->key() == Qt::Key_Up) {
//    if (navigation.GetCameraStatus() == kPanorama) {
//      navigation.MoveForwardPanorama();
//    }
//  } else if (e->key() == Qt::Key_Down) {
//    if (navigation.GetCameraStatus() == kPanorama) {
//      navigation.MoveBackwardPanorama();
//    }
//  } else if (e->key() == Qt::Key_Left) {
//    double rotation_angle = kRotationAngle;

//    bool slow = false;
//    if(e->modifiers() & Qt::ShiftModifier &&
//       e->modifiers() & Qt::AltModifier) {
//      rotation_angle = 90.0 * M_PI / 180.0;
//      slow = true;
//    } else if(e->modifiers() & Qt::ShiftModifier) {
//      rotation_angle = 10.0 * M_PI / 180.0;
//    } else if(e->modifiers() & Qt::AltModifier) {
//      rotation_angle = 90.0 * M_PI / 180.0;
//    }
//    if (navigation.GetCameraStatus() == kPanorama) {
//      navigation.RotatePanorama(rotation_angle);
//    } else if (navigation.GetCameraStatus() == kAir ||
//               navigation.GetCameraStatus() == kTree) {
//      navigation.RotateAir(-rotation_angle, slow);
//    } else if (navigation.GetCameraStatus() == kFloorplan) {
////      navigation.RotateFloorplan(-rotation_angle);
//    }
//  }
//  else if (e->key() == Qt::Key_Right) {
//    double rotation_angle = kRotationAngle;
//    bool slow = false;
//    if(e->modifiers() & Qt::ShiftModifier &&
//       e->modifiers() & Qt::AltModifier) {
//      rotation_angle = 90.0 * M_PI / 180.0;
//      slow = true;
//    } else if(e->modifiers() & Qt::ShiftModifier) {
//      rotation_angle = 10.0 * M_PI / 180.0;
//    } else if(e->modifiers() & Qt::AltModifier) {
//      rotation_angle = 90.0 * M_PI / 180.0;
//    }
    
//    if (navigation.GetCameraStatus() == kPanorama) {
//      navigation.RotatePanorama(-rotation_angle);
//    }
//  }
//  //----------------------------------------------------------------------
//  // 4 modes.
//  else if (e->key() == Qt::Key_A) {
//      cerr << "Key press event error." << endl;
//      exit(1);
//    if (navigation.GetCameraStatus() == kAir)
//      navigation.AirToPanorama(navigation.GetCameraPanorama().start_index);
//    else if (navigation.GetCameraStatus() == kFloorplan)
//      navigation.FloorplanToPanorama(navigation.GetCameraPanorama().start_index);
//  } else if (e->key() == Qt::Key_S) {
//    if (navigation.GetCameraStatus() == kPanorama)
//      navigation.PanoramaToAir();
//    else if (navigation.GetCameraStatus() == kFloorplan)
//      navigation.FloorplanToAir();
//    else if (navigation.GetCameraStatus() == kTree)
//      navigation.TreeToAir();
//  } else if (e->key() == Qt::Key_D) {
//    if (navigation.GetCameraStatus() == kPanorama)
//      navigation.PanoramaToFloorplan();
//    else if (navigation.GetCameraStatus() == kAir)
//      navigation.AirToFloorplan();
//  } else if (e->key() == Qt::Key_F) {
//    if (navigation.GetCameraStatus() == kAir) {
//      navigation.AirToTree();
//      tree_entry_time = object_animation_time.elapsed();
//    }
//  }
//  //----------------------------------------------------------------------
//  // Toggle switch.
//  else if (e->key() == Qt::Key_R) {
//    indoor_polygon_renderer.ToggleRenderMode();
//    updateGL();
//  } else if (e->key() == Qt::Key_O) {
//    object_renderer.Toggle();
//    updateGL();
//  } else if (e->key() == Qt::Key_P) {
//    polygon_or_indoor_polygon = !polygon_or_indoor_polygon;
//    updateGL();
//  } else if (e->key() == Qt::Key_T) {
//    render_backface = !render_backface;
//    updateGL();
//  } else if (e->key() == Qt::Key_U) {
//    if (background == kBackgroundBlack)
//      background = kBackgroundWhite;
//    else
//      background = kBackgroundBlack;

//    glClearColor(background[0], background[1], background[2], 0);
//    updateGL();
//  } else if (e->key() == Qt::Key_I) {
//    cerr << "filename > " << flush;
//    string filename;
//    cin >> filename;
    
//    glReadBuffer(GL_BACK);
//    vector<unsigned char> buffer(width() * height() * 4);
//    glReadPixels(0, 0, width(), height(), GL_RGBA, GL_UNSIGNED_BYTE, &buffer[0]);
//    cv::Mat image(height(), width(), CV_8UC3);
//    for (int y = 0; y < height(); ++y) {
//      for (int x = 0; x < width(); ++x) {
//        image.at<cv::Vec3b>(height() - 1 - y, x) = cv::Vec3b(buffer[4 * (y * width() + x) + 2],
//                                                             buffer[4 * (y * width() + x) + 1],
//                                                             buffer[4 * (y * width() + x) + 0]);
//      }
//    }
//    cv::imwrite(filename, image);
//  }
}

void MainWidget::keyReleaseEvent(QKeyEvent *) {  
}

void MainWidget::wheelEvent(QWheelEvent* e) {
    if (e->delta() > 0)
        viewing_angle_y = max(0.0, viewing_angle_y - 1);
//        depth_map_renderer.increaseViewScale();
    else if (e->delta() < 0)
        viewing_angle_y = min(179.9, viewing_angle_y + 1);
//    cout << viewing_angle_y << endl;
//        depth_map_renderer.decreaseViewScale();
    updateGL();
}

void MainWidget::mouseMoveEvent(QMouseEvent *e) {
  QVector2D diff = QVector2D(e->localPos()) - mouseMovePosition;
  mouseMovePosition = QVector2D(e->localPos());


  if ((e->buttons() & Qt::LeftButton) != 0) {
      depth_map_renderer.rotateByY(-diff.x() * 0.3);
      depth_map_renderer.rotateByX(-diff.y() * 0.3);
  }
}

void MainWidget::timerEvent(QTimerEvent *) {

    const double TIME_FOR_EACH_DIRECTION = 3;
    const double MOVEMENT_FOR_EACH_DIRECTION = 0.002;

    const double TIME_FOR_SINGLE_MOVEMENT = 1;
//    cout << "timer event" << endl;
    if (transition_progress < 1.0) {
        switch (transition_direction) {
        case 'U':
            transition_progress += 1 / (TIME_FOR_SINGLE_MOVEMENT * 60);
            depth_map_renderer.moveByY(MOVEMENT_FOR_EACH_DIRECTION / 2 / (TIME_FOR_SINGLE_MOVEMENT * 60));
            break;
        case 'D':
            transition_progress += TIME_FOR_SINGLE_MOVEMENT / 60;
            depth_map_renderer.moveByY(-MOVEMENT_FOR_EACH_DIRECTION / 2 / (TIME_FOR_SINGLE_MOVEMENT * 60));
            break;
        case 'L':
            transition_progress += TIME_FOR_SINGLE_MOVEMENT / 60;
            depth_map_renderer.moveByX(MOVEMENT_FOR_EACH_DIRECTION / 2 / (TIME_FOR_SINGLE_MOVEMENT * 60));
            break;
        case 'R':
            transition_progress += TIME_FOR_SINGLE_MOVEMENT / 60;
            depth_map_renderer.moveByX(-MOVEMENT_FOR_EACH_DIRECTION / 2 / (TIME_FOR_SINGLE_MOVEMENT * 60));
            break;
        case 'F':
            transition_progress += TIME_FOR_SINGLE_MOVEMENT / 60;
            depth_map_renderer.moveByZ(MOVEMENT_FOR_EACH_DIRECTION / 2 / (TIME_FOR_SINGLE_MOVEMENT * 60));
            break;
        case 'B':
            transition_progress += TIME_FOR_SINGLE_MOVEMENT / 60;
            depth_map_renderer.moveByZ(-MOVEMENT_FOR_EACH_DIRECTION / 2 / (TIME_FOR_SINGLE_MOVEMENT * 60));
            break;
        case 'P':
//            cout << "yes" << endl;
            if (transition_progress < 1.0 / 12)
                depth_map_renderer.moveByX(MOVEMENT_FOR_EACH_DIRECTION / (TIME_FOR_EACH_DIRECTION / 2 * 60));
            else if (transition_progress < 1.0 / 12 * 3)
                depth_map_renderer.moveByX(-MOVEMENT_FOR_EACH_DIRECTION / (TIME_FOR_EACH_DIRECTION / 2 * 60));
            else if (transition_progress < 1.0 / 12 * 4)
                depth_map_renderer.moveByX(MOVEMENT_FOR_EACH_DIRECTION / (TIME_FOR_EACH_DIRECTION / 2 * 60));
            else if (transition_progress < 1.0 / 12 * 5)
                depth_map_renderer.moveByY(MOVEMENT_FOR_EACH_DIRECTION / (TIME_FOR_EACH_DIRECTION / 2 * 60));
            else if (transition_progress < 1.0 / 12 * 7)
                depth_map_renderer.moveByY(-MOVEMENT_FOR_EACH_DIRECTION / (TIME_FOR_EACH_DIRECTION / 2 * 60));
            else if (transition_progress < 1.0 / 12 * 8)
                depth_map_renderer.moveByY(MOVEMENT_FOR_EACH_DIRECTION / (TIME_FOR_EACH_DIRECTION / 2 * 60));
            else if (transition_progress < 1.0 / 12 * 9)
                depth_map_renderer.moveByZ(MOVEMENT_FOR_EACH_DIRECTION / (TIME_FOR_EACH_DIRECTION / 2 * 60));
            else if (transition_progress < 1.0 / 12 * 11)
                depth_map_renderer.moveByZ(-MOVEMENT_FOR_EACH_DIRECTION / (TIME_FOR_EACH_DIRECTION / 2 * 60));
            else if (transition_progress < 1.0 / 12 * 12)
                depth_map_renderer.moveByZ(MOVEMENT_FOR_EACH_DIRECTION / (TIME_FOR_EACH_DIRECTION / 2 * 60));
            transition_progress += 1.0 / (TIME_FOR_EACH_DIRECTION * 6 * 60);
            break;
        default:
            break;
        }
    }
    updateGL();
}  

bool MainWidget::RightAfterSimpleClick(const double margin) const {
  const double kDoubleClickMargin = 0.5;
  if ((simple_click_time.elapsed() - simple_click_time_offset_by_move) / 1000.0 >
      kFadeInSeconds - margin &&
      (simple_click_time.elapsed() - simple_click_time_offset_by_move) / 1000.0 <
      kFadeOutSeconds + margin &&
      (double_click_time.elapsed() -
       (simple_click_time.elapsed() - simple_click_time_offset_by_move)) / 1000.0 >
      kDoubleClickMargin - margin) {
    return true;
  }
  else {
    return false;
  }
}

  /*
double MainWidget::Progress() {
  return ProgressFunction(simple_click_time.elapsed() / 1000.0,
                          simple_click_time_offset_by_move / 1000.0,
                          kFadeInSeconds,
                          kFadeOutSeconds);
}
  */

//double MainWidget::AnimationTrapezoid() {
//  return AnimationTrapezoidUtil(simple_click_time.elapsed() / 1000.0,
//                                simple_click_time_offset_by_move / 1000.0,
//                                kFadeInSeconds,
//                                kFadeOutSeconds);
//}

//double MainWidget::AnimationLinear() {
//  return AnimationLinearUtil(simple_click_time.elapsed() / 1000.0,
//                             simple_click_time_offset_by_move / 1000.0,
//                             kFadeInSeconds,
//                             kFadeOutSeconds);
//}

  
}  // namespace structured_indoor_modeling
