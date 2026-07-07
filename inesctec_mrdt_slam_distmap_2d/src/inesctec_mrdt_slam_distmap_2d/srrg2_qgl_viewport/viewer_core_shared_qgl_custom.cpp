#include "inesctec_mrdt_slam_distmap_2d/srrg2_qgl_viewport/viewer_core_shared_qgl_custom.hpp"

namespace srrg2_qgl_viewport
{

void ViewerCoreSharedQGLCustom::lookAtPosition2D(const std::string& canvas_name,
                                                 float x, float y)
{
  srrg2_core::ViewerCoreBase::StringCanvasViewsMap::iterator it =
      this->_canvas_viewport_map.find(canvas_name);

  if (it == this->_canvas_viewport_map.end())
  {
    throw std::runtime_error(
        "ViewerCoreSharedQGLCustom::lookAtPosition2D | canvas '" + canvas_name +
        "' not found in the canvas - viewport association map");
  }

  srrg2_core::ViewerCoreBase::CanvasViews* canvas_view = it->second;

  for (size_t idx = 0; idx < canvas_view->num_views; ++idx)
  {
    srrg2_qgl_viewport::QGLViewport* v =
        static_cast<srrg2_qgl_viewport::QGLViewport*>(
            canvas_view->viewports[idx]);

    if ((!canvas_view->viewports[idx]) || (!v->camera()))
    {
      continue;
    }

    v->camera()->setPosition(qglviewer::Vec(x, y, v->camera()->position().z));
    // v->camera()->setSceneCenter(qglviewer::Vec(1.0f, 1.0f, 0.0f));
    // v->camera()->setUpVector(qglviewer::Vec(0.0f, 0.0f, 0.0f));

    v->camera()->lookAt(qglviewer::Vec(x, y, 0.f));
  }
}

}  // namespace srrg2_qgl_viewport
