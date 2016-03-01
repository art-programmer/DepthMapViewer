#ifndef MAIN_WIDGET_H__
#define MAIN_WIDGET_H__

#include <QGLWidget>
//#include <QGLFunctions>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QTime>

#include <map>
#include <string>
#include <vector>

#include "file_io.h"
#include "configuration.h"
#include "depth_map_renderer.h"
#include "panorama.h"
#include "view_parameters.h"

namespace structured_indoor_modeling {
  
class MainWidget : public QGLWidget, protected QOpenGLFunctions {
Q_OBJECT
public:
    explicit MainWidget(const Configuration& configuration,
                        const std::string& suffix,
                        QWidget *parent = 0);
    ~MainWidget();

    void setRenderingMode(const char mode);


protected:
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseDoubleClickEvent(QMouseEvent *e);
//    void keyPressEvent(QKeyEvent *e);
//    void keyReleaseEvent(QKeyEvent *e);
    void wheelEvent(QWheelEvent* e);
    
    void timerEvent(QTimerEvent *e);

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

private:
    FileIO file_io;
    //----------------------------------------------------------------------
    // Core data.
//    std::vector<Panorama> panoramas;  // No image data loaded.
//    std::vector<Panorama> depth_maps;
    double image_width;
    double image_height;

    //----------------------------------------------------------------------
    // Renderers.
    DepthMapRenderer depth_map_renderer;
    // Tree organizer.
    ViewParameters view_parameters;
//    // Navigation knows the entire state of the viewer.
//    Navigation navigation;
    // Configuration holds some parameters.
    Configuration configuration;

    //----------------------------------------------------------------------
    // GL resources.
    GLuint frameids[2];
    GLuint texids[2];
    GLuint renderids[2];
    int current_width;
    int current_height;

    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    // GUI states.
    double prev_animation_linear;
    double prev_animation_trapezoid;    
    bool fresh_screen_for_panorama;
    bool fresh_screen_for_air;
    bool fresh_screen_for_floorplan;
    double simple_click_time_offset_by_move;

    bool polygon_or_indoor_polygon;
    
    QBasicTimer timer;
    QVector2D mousePressPosition;
    QVector2D mouseMovePosition;

    QOpenGLShaderProgram blend_program;
    QOpenGLShaderProgram depth_map_program;

    QTime simple_click_time;
    QTime double_click_time;
    QTime object_animation_time;
    bool mouse_down;
    int tree_entry_time;

    void FreeResources();
    void AllocateResources();
    void SetMatrices();

    void InitPanoramasPanoramaRenderers();
    // void RenderQuad(const double alpha);
    void InitializeShaders();
   
    // Keep rendering after no action for a while.
    bool RightAfterSimpleClick(const double margin) const;

    // After a single click, where we are between kFadeInSeconds and
    // kFadeOutSeconds.
    // double Progress();
    double AnimationTrapezoid();
    double AnimationLinear();

    std::map<int, int> panorama_to_room;
    std::map<int, int> room_to_panorama;
    std::vector<std::vector<double> > panorama_distance_table;
    bool render_backface;
    
    static const double kRenderMargin;
    static const double kFadeInSeconds;
    static const double kFadeOutSeconds;
    static const Eigen::Vector3f kBackgroundBlack;
    static const Eigen::Vector3f kBackgroundWhite;
    Eigen::Vector3f background;


    double viewing_angle_y;

    double transition_progress;
    char transition_direction;

    //----------------------------------------------------------------------
    // Paint functions.
    void PaintPanorama();
    void PaintAir();
    void PaintFloorplan();
    void PaintTree();
    
    //----------------------------------------------------------------------
    // Render functions.
    void RenderFloorplan(const double alpha,
                         const bool emphasize,
                         const double height_adjustment);
    void RenderPanorama(const double alpha);
    void RenderObjects(const double alpha);
    void RenderPolygon(const int room_not_rendered,
                       const double alpha,
                       const double height_adjustment,
                       const bool depth_order_height_adjustment,
                       const int room_highlighted);
    void RenderTexturedPolygon(const double alpha);
    void RenderPolygonLabels(const int room_not_rendered,
                             const double height_adjustment,
                             const bool depth_order_height_adjustment);
    void RenderFloorplanLabels();
    void RenderThumbnail(const double alpha,
                         const int room_highlighted,
                         QGLWidget* qgl_widget);
    void RenderAllThumbnails(const double alpha,
                             const int room_highlighted,
                             QGLWidget* qgl_widget);
    void RenderAllRoomNames(const double alpha,
                            QGLWidget* qgl_widget);
    
    void RenderPanoramaTransition(const int start_index,
                                  const int end_index,
                                  const double start_weight);    
    void BlendFrames(const double weight, const int divide_by_alpha_mode);
    void RenderPanoramaTour();    
    void RenderPanoramaToAirTransition(const bool flip = false);
    void RenderPanoramaToFloorplanTransition(const bool flip = false);
    void RenderAirToFloorplanTransition(const bool flip = false);
    void RenderTree(const double air_to_tree_progress);
    void RenderTreeTop(const double air_to_tree_progress,
                       const double animation,
                       const double animation_alpha,
                       const Eigen::Vector3d& offset_direction,
                       const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >& top_lines);
    void RenderTreeMiddle(const double air_to_tree_progress,
                          const double animation,
                          const double animation_alpha,
                          const std::vector<std::vector<Eigen::Vector3d> >& boundaries,
                          const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >& bottom_lines);
    void RenderTreeBottom(const double air_to_tree_progress,
                          const double animation,
                          const Eigen::Vector3d& offset_direction);
    
    int FindRoomHighlighted(const Eigen::Vector2i& pixel);

    //void SetTreeRenderingParameters();

    double ObjectAnimationPosition() const;
    bool ObjectAnimation() const;
    
    void ClearDisplay();
    void ClearDisplayWithWhite();

public slots:
    void startMovement();
    void resetScene(const int scene_index);
};

}  // namespace structured_indoor_modeling

#endif // MAIN_WIDGET_H__
