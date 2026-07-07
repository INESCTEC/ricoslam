#pragma once

#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>

#include <exception>

namespace srrg2_qgl_viewport
{

/**
 * @brief custom viewer core shared QGL class to allow access to protected
 *        methods and variables of the parent class ViewerCoreSharedQGL
 *
 * _canvas_viewport_map required to get the viewports associated to a canvas
 * then the viewport can be used to set the camera position
 */
class ViewerCoreSharedQGLCustom : public ViewerCoreSharedQGL
{
 public:

  /**
   * @brief Construct a new ViewerCoreSharedQGLCustom object
   *
   * @param[in] argc_ number of input arguments of the executable in the main
   *                  thread
   * @param[in] argv_ array of strings corresponding to the input arguments of
   *                  the executable in the main thread
   * @param[in] qapp_ QApplication pointer (common to all canvas and viewports)
   * @param[in] buffer_size_ OpenGL buffer size
   * @param[in] buffers_num_ OpenGL buffer number
   * @param[in] opengl_frame_time_ms_ frame time for the OpenGL context to limit
   *                                  the FPS and avoid CPU melting
   * @param[in] install_signal_handler_ flag to install a signal handler
   */
  ViewerCoreSharedQGLCustom(int argc_, char** argv_, QApplication* qapp_,
                            const size_t& buffer_size_ = BUFFER_SIZE_50MEGABYTE,
                            const size_t& buffers_num_ = 3,
                            const size_t& opengl_frame_time_ms_ = 25,
                            const bool install_signal_handler_ = false)
      : ViewerCoreSharedQGL(argc_, argv_, qapp_, buffer_size_, buffers_num_,
                            opengl_frame_time_ms_, install_signal_handler_)
  {
  }

  /**
   * @brief Destroy the ViewerCoreSharedQGLCustom object
   */
  virtual ~ViewerCoreSharedQGLCustom() {}

  /**
   * @brief Look at a 2D position in the specified canvas
   * @param[in] canvas_name canvas title name string
   * @param[in] x x coordinate of the position to look at (m)
   * @param[in] y y coordinate of the position to look at (m)
   */
  void lookAtPosition2D(const std::string& canvas_name, float x, float y);

 private:

  ViewerCoreSharedQGLCustom() = delete;
};  // class ViewerCoreSharedQGLCustom

}  // namespace srrg2_qgl_viewport
