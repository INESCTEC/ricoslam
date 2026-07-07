#pragma once

#include <chrono>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <unordered_map>

// Eigen
#include <Eigen/Eigenvalues>

// SRRG Software
#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry3d.h>
#include <srrg2_core/srrg_pcl/point_types.h>
#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_core/srrg_viewer/viewer_core/color_palette.h>
#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg2_solver/solver_core/factor.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky.h>
#include <srrg2_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg2_solver/solver_core/robustifier.h>
#include <srrg2_solver/solver_core/robustifier_policy.h>
#include <srrg2_solver/solver_core/solver.h>
#include <srrg2_solver/solver_core/termination_criteria.h>
#include <srrg2_solver/solver_core/variable.h>

#include "inesctec_mrdt_slam_distmap_2d/pcl/point_types.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_qgl_viewport/viewer_core_shared_qgl_custom.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_prior_error_factor.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/solver_actions.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

/**
 * @brief aligner base class
 */
class AlignerBase
{
 public:

  using SolverType = srrg2_solver::Solver;
  using FactorGraphType = srrg2_solver::FactorGraph;

  using IterationAlgorithm = srrg2_solver::IterationAlgorithmGN;
  using LinearSolver = srrg2_solver::SparseBlockLinearSolverCholesky;

  using OGLAppType = QApplication;
  using OGLAppTypePtr = std::shared_ptr<OGLAppType>;
  using OGLViewerType = srrg2_qgl_viewport::ViewerCoreSharedQGLCustom;
  using OGLViewerTypePtr = std::shared_ptr<OGLViewerType>;
  using OGLCanvasType = srrg2_core::ViewerCanvas;
  using OGLCanvasTypePtr = std::shared_ptr<OGLCanvasType>;

 public:

  /**
   * @brief aligner base parametrization (solver configuration)
   */
  struct Param
  {
    bool m_debug;            //!< enable debug mode (OpenGL-based visualization)
    bool m_solver_verbose;   //!< enable verbose mode of the solver (to show the
                             //!< iteration stats)
    float m_solver_damping;  //!< damping parameter for the iteration algorithm
                             //!< when computing the new state
    int m_solver_max_iterations;  //!< maximum number of iterations allowed for
                                  //!< the solver
    std::string m_solver_criteria_type;  //!< type of termination criteria
                                         //!< (simple, gradient, stepnorm)
                                         //!< (empty to force max iterations)
    float m_solver_criteria_epsilon;     //!< epsilon threshold for the solver's
                                         //!< termination criteria
    std::string
        m_solver_robustifier_type;  //!< type of robustifier (saturated, cauchy,
                                    //!< clamp, huber) (empty not used)
    int m_solver_robustifier_num_iterations_coarse;  //!< number of iterations
                                                     //!< for coarse matching (0
                                                     //!< to disable coarse +
                                                     //!< fine)
    float m_solver_robustifier_threshold_coarse;  //!< robustifier threshold to
                                                  //!< apply to the error
                                                  //!< factors (coarse matching)
    float m_solver_robustifier_threshold_fine;    //!< robustifier threshold to
                                                //!< apply to the error factors
                                                //!< (fine matching)
    float m_failure_ratio_inliers;  //!< maximum ratio inliers threshold to
                                    //!< trigger the failure status of the
                                    //!< aligner ([0..1[; -1 not used)
    float m_failure_chi_inliers;    //!< mean inliers chi error threshold to
                                    //!< trigger the failure status of the
                                    //!< aligner ([0..[; -1 not used)
    int64_t m_dbg_ogl_interval;     //!< sleep interval between consecutive
                                    //!< visualizations (ms)
    std::string m_dbg_ogl_mode;     //!< type of debug visualization mode
                                    //!< (iterationend|computeend)
    bool m_dbg_ogl_show_correspondences;  //!< show moving-fixed correspondences
    bool m_dbg_ogl_show_normals;          //!< enable normals visualization

    /**
     * @brief default constructor for a new Param object
     */
    Param()
        : m_debug(false),
          m_solver_verbose(false),
          m_solver_damping(1.f),
          m_solver_max_iterations(25),
          m_solver_criteria_type("stepnorm"),
          m_solver_criteria_epsilon(1e-5),
          m_solver_robustifier_type("cauchy"),
          m_solver_robustifier_num_iterations_coarse(10),
          m_solver_robustifier_threshold_coarse(0.025f),
          m_solver_robustifier_threshold_fine(0.0025f),
          m_failure_ratio_inliers(0.35f),
          m_failure_chi_inliers(-1.0f),
          m_dbg_ogl_interval(0),
          m_dbg_ogl_mode(""),
          m_dbg_ogl_show_correspondences(false),
          m_dbg_ogl_show_normals(false)
    {
    }

    /**
     * @brief Construct a new Param object
     * @param[in] debug enable debug mode (OpenGL-based visualization)
     * @param[in] solver_verbose enable verbose mode of the solver (to show the
     *                           iteration stats)
     * @param[in] solver_damping damping parameter for the iteration algorithm
     *                           when computing the new state
     * @param[in] solver_max_iterations maximum number of iterations allowed
     *                                  for the solver
     * @param[in] solver_criteria_type type of termination criteria
     *                                 (simple, gradient, stepnorm)
     * @param[in] solver_criteria_epsilon epsilon threshold for the solver's
     *                                    termination criteria
     * @param[in] solver_robustifier_type type of robustifier
     *                                    (saturated, cauchy, clamp, huber)
     *                                    (empty not used)
     * @param[in] solver_robustifier_num_iterations_coarse number of iterations
     *                                                     for coarse matching
     *                                                     (0 to disable
     *                                                     coarse + fine)
     * @param[in] solver_robustifier_threshold_coarse robustifier threshold to
     *                                                apply to the error factors
     *                                                (coarse matching)
     * @param[in] solver_robustifier_threshold_fine robustifier threshold to
     *                                              apply to the error factors
     *                                              (fine matching)
     * @param[in] failure_ratio_inliers maximum ratio inliers threshold to
     *                                  trigger the failure status of the
     *                                  aligner ([0..1[; -1 not used)
     * @param[in] failure_chi_inliers mean inliers chi error threshold to
     *                                trigger the failure status of the aligner
     *                                ([0..[; -1 not used)
     * @param[in] dbg_ogl_interval sleep interval between consecutive
     *                             visualizations (ms)
     * @param[in] dbg_ogl_mode type of debug visualization mode
     *                         (iterationend|computeend)
     * @param[in] dbg_ogl_show_correspondences show moving-fixed correspondences
     * @param[in] dbg_ogl_show_normals enable normals visualization
     */
    Param(bool debug, bool solver_verbose, float solver_damping,
          int solver_max_iterations, const std::string& solver_criteria_type,
          float solver_criteria_epsilon,
          const std::string& solver_robustifier_type,
          int solver_robustifier_num_iterations_coarse,
          float solver_robustifier_threshold_coarse,
          float solver_robustifier_threshold_fine, float failure_ratio_inliers,
          float failure_chi_inliers, int64_t dbg_ogl_interval,
          const std::string& dbg_ogl_mode, bool dbg_ogl_show_correspondences,
          bool dbg_ogl_show_normals)
        : m_debug(debug),
          m_solver_verbose(solver_verbose),
          m_solver_damping(solver_damping),
          m_solver_max_iterations(solver_max_iterations),
          m_solver_criteria_type(solver_criteria_type),
          m_solver_criteria_epsilon(solver_criteria_epsilon),
          m_solver_robustifier_type(solver_robustifier_type),
          m_solver_robustifier_num_iterations_coarse(
              solver_robustifier_num_iterations_coarse),
          m_solver_robustifier_threshold_coarse(
              solver_robustifier_threshold_coarse),
          m_solver_robustifier_threshold_fine(
              solver_robustifier_threshold_fine),
          m_failure_ratio_inliers(failure_ratio_inliers),
          m_failure_chi_inliers(failure_chi_inliers),
          m_dbg_ogl_interval(dbg_ogl_interval),
          m_dbg_ogl_mode(dbg_ogl_mode),
          m_dbg_ogl_show_correspondences(dbg_ogl_show_correspondences),
          m_dbg_ogl_show_normals(dbg_ogl_show_normals)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, const Param& param)
    {
      os << "- OpenGL-based visualization dbg: "
         << (param.m_debug ? "enabled" : "not enabled") << std::endl;
      os << "- solver verbose enabled   : "
         << (param.m_solver_verbose ? "enabled" : "not enabled") << std::endl;
      os << "- solver algorithm damping : " << param.m_solver_damping
         << std::endl;
      os << "- solver maximum iterations: " << param.m_solver_max_iterations
         << std::endl;
      os << "- solver criteria type   : "
         << (param.m_solver_criteria_type.empty()
                 ? "not enabled"
                 : param.m_solver_criteria_type)
         << std::endl;
      if (!param.m_solver_criteria_type.empty())
      {
        os << "- solver criteria epsilon: " << param.m_solver_criteria_epsilon
           << std::endl;
      }
      os << "- solver robustifier type                       : "
         << (param.m_solver_robustifier_type.empty()
                 ? "not enabled"
                 : param.m_solver_robustifier_type.c_str())
         << std::endl;
      if (!param.m_solver_robustifier_type.empty())
      {
        os << "- solver robustifier coarse number of iterations: ";
        if (param.m_solver_robustifier_num_iterations_coarse > 0)
        {
          os << param.m_solver_robustifier_num_iterations_coarse << std::endl;
          os << "- solver robustifier threshold coarse           : "
             << param.m_solver_robustifier_threshold_coarse << std::endl;
        }
        else
        {
          os << "not enabled" << std::endl;
          os << "- solver robustifier threshold coarse           : "
             << "not enabled" << std::endl;
        }
        os << "- solver robustifier threshold fine             : "
           << param.m_solver_robustifier_threshold_fine << std::endl;
      }
      else
      {
        os << "- solver robustifier coarse number of iterations: "
           << "not enabled" << std::endl;
        os << "- solver robustifier threshold coarse           : "
           << "not enabled" << std::endl;
        os << "- solver robustifier threshold fine             : "
           << "not enabled" << std::endl;
      }
      if (param.m_failure_ratio_inliers > 0)
      {
        os << "- failure inliers ratio: " << param.m_failure_ratio_inliers
           << std::endl;
      }
      else
      {
        os << "- failure inliers ratio: not enabled" << std::endl;
      }
      if (param.m_failure_chi_inliers > 0)
      {
        os << "- failure chi inliers  : " << param.m_failure_chi_inliers
           << std::endl;
      }
      else
      {
        os << "- failure chi inliers  : not enabled" << std::endl;
      }
      if (param.m_debug)
      {
        os << "- dbg sleep interval between iterations (ms): "
           << param.m_dbg_ogl_interval << std::endl;
        os << "- OpenGL-based visualization debug mode     : "
           << param.m_dbg_ogl_mode << std::endl;
        os << "- OpenGL-based visualization correspondences: "
           << (param.m_dbg_ogl_show_correspondences ? "enabled" : "not enabled")
           << std::endl;
        os << "- OpenGL-based visualization show normals   : "
           << (param.m_dbg_ogl_show_normals ? "enabled" : "not enabled")
           << std::endl;
      }
      else
      {
        os << "- dbg sleep interval between iterations (ms): " << "not enabled"
           << std::endl;
        os << "- OpenGL-based visualization debug mode     : " << "not enabled"
           << std::endl;
        os << "- OpenGL-based visualization correspondences: N/A" << std::endl;
        os << "- OpenGL-based visualization show normals   : N/A" << std::endl;
      }

      return os;
    }  // friend std::ostream& operator<<(std::ostream& os, const StatsShort&
       // stats)

  };  // struct AlignerBase::Param

 public:

  /**
   * @brief stats struct for the aligner
   *        (can be used for loop closure detection, tracking, etc.)
   */
  struct Stats
  {
    int m_num_iterations;    //!< number of iterations of the solver
    float m_num_moving_pts;  //!< number of moving points
    float m_num_fixed_pts;   //!< number of fixed points
    float m_num_inliers;     //!< number of inliers
    float m_num_outliers;    //!< number of outliers
    float m_num_kernelized;  //!< number of points kernelized by the robustifier
    float m_chi_inliers;     //!< mean chi error of the inliers
    float m_chi_outliers;    //!< mean chi error of the outliers
    float m_chi_kernelized;  //!< mean chi error of the points kernelized by the
                             //!< robustifier
    float m_inliers_ratio;   //!< inliers ratio
    float m_outliers_ratio;  //!< outliers ratio
    float m_suppressed_ratio;  //!< suppressed ratio

    friend std::ostream& operator<<(std::ostream& os, const Stats& stats)
    {
      os << "num_iterations: " << stats.m_num_iterations
         << "; num_moving_pts: " << stats.m_num_moving_pts
         << "; num_fixed_pts: " << stats.m_num_fixed_pts
         << "; num_inliers: " << stats.m_num_inliers
         << "; num_outliers: " << stats.m_num_outliers
         << "; num_kernelized: " << stats.m_num_kernelized
         << "; chi_inliers: " << stats.m_chi_inliers
         << "; chi_outliers: " << stats.m_chi_outliers
         << "; chi_kernelized: " << stats.m_chi_kernelized
         << "; inliers_ratio: " << stats.m_inliers_ratio
         << "; outliers_ratio: " << stats.m_outliers_ratio
         << "; suppressed_ratio: " << stats.m_suppressed_ratio;
      return os;
    }  // friend std::ostream& operator<<(std::ostream& os, const Stats& stats)

  };  // struct AlignerBase::Stats

  /**
   * @brief short stats struct for the aligner
   *        (...mostly, for stdout debug...)
   */
  struct StatsShort
  {
    float m_chi_inliers;       //!< mean chi error of the inliers
    float m_chi_outliers;      //!< mean chi error of the outliers
    float m_chi_kernelized;    //!< mean chi error of the points kernelized by
                               //!< the robustifier
    float m_inliers_ratio;     //!< inliers ratio
    float m_outliers_ratio;    //!< outliers ratio
    float m_suppressed_ratio;  //!< suppressed ratio

    friend std::ostream& operator<<(std::ostream& os, const StatsShort& stats)
    {
      os << "chi_inliers: " << stats.m_chi_inliers
         << "; chi_outliers: " << stats.m_chi_outliers
         << "; chi_kernelized: " << stats.m_chi_kernelized
         << "; inliers_ratio: " << stats.m_inliers_ratio
         << "; outliers_ratio: " << stats.m_outliers_ratio
         << "; suppressed_ratio: " << stats.m_suppressed_ratio;
      return os;
    }  // friend std::ostream& operator<<(std::ostream& os, const StatsShort&
       // stats)

  };  // struct AlignerBase::StatsShort

  /**
   * @brief status enumeration for the aligner
   */
  enum Status
  {
    Success = 0,                //!< success alignment
    NotEnoughInliersRatio = 1,  //!< not enough inliers
    NotEnoughChiInliers = 2,    //!< high inliers chi error
    Fail = 3                    //!< alignment failure (e.g., bad coefficients)
  };  // enum AlignerBase::Status

 protected:

  Param m_param_;    //!< parametrization
  Status m_status_;  //!< status
  Stats m_stats_;    //!< stats

  SolverType m_solver_;      //!< solver to compute the alignment
  FactorGraphType m_graph_;  //!< factor graph

  OGLAppTypePtr m_dbg_ogl_qapp_;       //!< Qt application required for the SRRG
                                       //!< ViewerCoreSharedQGL viz
  OGLViewerTypePtr m_dbg_ogl_viewer_;  //!< ViewerCoreSharedQGL handler
  OGLCanvasTypePtr
      m_dbg_ogl_canvas_;  //!< viz canvas to show the tracking results
  std::string m_dbg_ogl_canvas_name_;  //!< viz canvas title name (to be used in
                                       //!< the ViewerCoreSharedQGL)

 public:

  /**
   * @brief Construct a new AlignerBase object
   * @param[in] param aligner parametrization
   */
  AlignerBase(const Param& param = Param())
      : m_param_(param),
        m_status_(Status::Fail),
        m_dbg_ogl_qapp_(nullptr),
        m_dbg_ogl_viewer_(nullptr),
        m_dbg_ogl_canvas_(nullptr),
        m_dbg_ogl_canvas_name_("")
  {
    assertInit(m_param_);
    init();
  }  // AlignerBase(const Param& = Param())

  virtual ~AlignerBase() = default;

  /**
   * @brief compute the alignment
   */
  virtual void compute() = 0;

  /**
   * @brief compute only the stats (inliers, outliers, surppressed) based on the
   *        current variable estimation
   */
  virtual void computeStatsOnly() = 0;

  /**
   * @brief setup the debug OpenGL-based visualization
   * @param[in] qapp QApplication shared object
   * @param[in] viewer shared viewer master
   * @param[in] canvas OpenGL canvas
   * @param[in] canvas_name OpenGL canvas name
   */
  void setupDbgOGL(const OGLAppTypePtr& qapp, const OGLViewerTypePtr& viewer,
                   const OGLCanvasTypePtr& canvas = nullptr,
                   const std::string& canvas_name = "AlignerBase")
  {
    if (!m_param_.m_debug)
    {
      throw std::runtime_error(
          "Aligner_::setupDbgOGL | "
          "debug mode not enabled (only w/ dbg enabled the OGL viewer can be "
          "setup)");
    }
    if (!qapp || !viewer)
    {
      throw std::runtime_error(
          "Aligner_::setupDbgOGL | "
          "QtApplication and OpenGL shared viewer shared pointers must be "
          "initialized");
    }

    m_dbg_ogl_qapp_ = qapp;
    m_dbg_ogl_viewer_ = viewer;
    m_dbg_ogl_canvas_name_ = canvas_name;

    if (!canvas)
    {
      m_dbg_ogl_canvas_ = m_dbg_ogl_viewer_->getCanvas(m_dbg_ogl_canvas_name_);
    }
    else
    {
      m_dbg_ogl_canvas_ = canvas;
      m_dbg_ogl_canvas_name_ = canvas->name();
    }
    m_dbg_ogl_canvas_->_setup();
  }  // void AlignerBase::setupDbgOGL(const OGLAppTypePtr&, const
     // OGLViewerTypePtr&, const std::string& = "AlignerBase")

  /**
   * @brief setup the debug OpenGL-based visualization
   * @param[in] argc number of input arguments of the executable in main thread
   * @param[in] argv array of strings corresponding to the input arguments of
   *                 the executable in the main thread
   * @param[in] qapp QApplication shared object
   * @param[in] canvas_name OpenGL canvas name
   * @param[in] viewer shared viewer master
   */
  void setupDbgOGL(int argc = 0, char* argv[] = nullptr,
                   const OGLAppTypePtr& qapp = nullptr,
                   const OGLViewerTypePtr& viewer = nullptr,
                   const std::string& canvas_name = "AlignerBase")
  {
    if (!m_param_.m_debug)
    {
      throw std::runtime_error(
          "Aligner_::setupDbgOGL | "
          "debug mode not enabled (only w/ dbg enabled the OGL viewer can be "
          "setup)");
    }
    if (!qapp)
    {
      m_dbg_ogl_qapp_ = std::make_shared<OGLAppType>(argc, argv);
    }
    else
    {
      m_dbg_ogl_qapp_ = qapp;
    }
    if (!viewer)
    {
      m_dbg_ogl_viewer_ =
          std::make_shared<OGLViewerType>(argc, argv, m_dbg_ogl_qapp_.get(),
                                          BUFFER_SIZE_50MEGABYTE, 3, 25, true);
    }
    else
    {
      m_dbg_ogl_viewer_ = viewer;
    }

    m_dbg_ogl_canvas_ = m_dbg_ogl_viewer_->getCanvas(canvas_name);
    m_dbg_ogl_canvas_name_ = canvas_name;

    m_dbg_ogl_canvas_->_setup();
  }  // void AlignerBase::setupDbgOGL(int = 0, char*[] = nullptr, const
     // OGLAppTypePtr& = nullptr, const OGLViewerTypePtr& = nullptr, const
     // std::string& = "AlignerBase")

  /**
   * @brief update the canvas for the debug OpenGL-based visualization
   */
  virtual void updateDbgOPGLCanvas(bool = true) {}

  /**
   * @brief set new parametrization for the aligner
   * @todo when debug = true, action should be added to the solver!
   *       (only done on the derived classes' init method...)
   * @param[in] param new parametrization
   */
  void setParam(const Param& param)
  {
    assertInit(param);
    m_param_ = param;
    init();
  }

  /**
   * @brief set a new maximum ratio inliers threshold
   *        (does not need to update the whole configuration)
   * @param[in] failure_ratio_inliers maximum ratio inliers threshold to trigger
   *                                  the failure status of the aligner
   *                                  ([0..1[; -1 not used)
   */
  inline void setParamFailureRatioInliers(float failure_ratio_inliers)
  {
    if (failure_ratio_inliers >= 1.f)
    {
      throw std::runtime_error(
          "AlignerBase::setParamFailureRatioInliers | if maximum inliers ratio "
          "threshold to trigger the failure status of the aligner enabled, "
          "ratio must be between [0..1[");
    }
    m_param_.m_failure_ratio_inliers = failure_ratio_inliers;
  }

  /**
   * @brief set a new mean inliers chi error threshold
   *        (does not need to update the whole configuration)
   * @param[in] failure_chi_inliers mean inliers chi error threshold to trigger
   *                                the failure status of the aligner
   *                                ([0..[; -1 not used)
   */
  inline void setParamFailureChiInliers(float failure_chi_inliers)
  {
    m_param_.m_failure_chi_inliers = failure_chi_inliers;
  }

 public:

  inline const Param& getParam() const { return m_param_; }
  inline Status getStatus() const { return m_status_; }
  inline const Stats& getStats() const { return m_stats_; }
  inline StatsShort getStatsShort() const
  {
    return StatsShort({m_stats_.m_chi_inliers, m_stats_.m_chi_outliers,
                       m_stats_.m_chi_kernelized, m_stats_.m_inliers_ratio,
                       m_stats_.m_outliers_ratio, m_stats_.m_suppressed_ratio});
  }  // inline StatsShort AlignerBase::getShortStats() const

  inline SolverType* getSolver() { return &this->m_solver_; }
  inline FactorGraphType* getGraph() { return &this->m_graph_; }
  inline bool isOpenGLDbg() const { return m_param_.m_debug; }

 protected:

  /**
   * @brief compute the aligner stats upon the execution of the
   *        Solver::compute method in AlignerBase::compute
   *
   * this function may be overloaded to compute the other missing stats here
   * and/or compute them in another way...
   */
  virtual void computeStats()
  {
    const srrg2_solver::IterationStats& solver_stats =
        m_solver_.iterationStats().back();

    m_stats_.m_num_iterations = solver_stats.iteration + 1;
    m_stats_.m_num_moving_pts = solver_stats.num_inliers +
                                solver_stats.num_outliers +
                                solver_stats.num_suppressed;
    m_stats_.m_num_inliers = solver_stats.num_inliers;
    m_stats_.m_num_outliers = solver_stats.num_outliers;
    m_stats_.m_num_kernelized = solver_stats.num_suppressed;
    m_stats_.m_chi_inliers =
        !solver_stats.num_inliers
            ? std::numeric_limits<float>::max()
            : solver_stats.chi_inliers / solver_stats.num_inliers;
    m_stats_.m_chi_outliers =
        !solver_stats.num_outliers
            ? std::numeric_limits<float>::max()
            : solver_stats.chi_outliers / solver_stats.num_outliers;
    m_stats_.m_chi_kernelized =
        !solver_stats.num_suppressed
            ? std::numeric_limits<float>::max()
            : solver_stats.chi_kernelized / solver_stats.num_suppressed;

    if (m_stats_.m_num_moving_pts > 0)
    {
      m_stats_.m_inliers_ratio =
          m_stats_.m_num_inliers / m_stats_.m_num_moving_pts;
      m_stats_.m_outliers_ratio =
          m_stats_.m_num_outliers / m_stats_.m_num_moving_pts;
      m_stats_.m_suppressed_ratio =
          m_stats_.m_num_kernelized / m_stats_.m_num_moving_pts;
    }
    else
    {
      m_stats_.m_inliers_ratio = 0.f;
      m_stats_.m_outliers_ratio = 0.f;
      m_stats_.m_suppressed_ratio = 0.f;
    }
  }  // virtual void AlignerBase::computeStats()

 private:

  static void assertInit(const Param& param)
  {
    if (param.m_solver_damping < 0)
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | solver algorithm "
          "damping value must be greater or equal to 0");
    }
    if (param.m_solver_max_iterations <= 0)
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | solver algorithm "
          "maximum iterations must be greater than 0");
    }
    if ((!param.m_solver_criteria_type.empty()) &&
        (param.m_solver_criteria_type.compare("simple") != 0) &&
        (param.m_solver_criteria_type.compare("gradient") != 0) &&
        (param.m_solver_criteria_type.compare("stepnorm") != 0))
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | invalid solver "
          "termination criteria type (simple, gradient, stepnorm)");
    }
    if ((!param.m_solver_criteria_type.empty()) &&
        (param.m_solver_criteria_epsilon < 0))
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | epsilon for the "
          "termination criteria must be greater or equal to 0 if termination "
          "criteria type enabled");
    }
    if ((!param.m_solver_robustifier_type.empty()) &&
        (param.m_solver_robustifier_type.compare("saturated") != 0) &&
        (param.m_solver_robustifier_type.compare("cauchy") != 0) &&
        (param.m_solver_robustifier_type.compare("clamp") != 0) &&
        (param.m_solver_robustifier_type.compare("huber") != 0))
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | invalid robustifier type "
          "(saturated, cauchy, clamp, huber)");
    }
    if ((!param.m_solver_robustifier_type.empty()) &&
        ((param.m_solver_robustifier_num_iterations_coarse < 0) ||
         (param.m_solver_robustifier_num_iterations_coarse >
          param.m_solver_max_iterations)))
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | if robustifier enabled, number of "
          "coarse iterations must be greater or equal to 0 and lower or equal "
          "to the maximum iterations");
    }
    if ((!param.m_solver_robustifier_type.empty()) &&
        ((param.m_solver_robustifier_threshold_fine <= 0) ||
         ((param.m_solver_robustifier_threshold_coarse <= 0) &&
          (param.m_solver_robustifier_num_iterations_coarse > 0))))
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | if robustifier enabled, "
          "thresholds must be greater than 0");
    }
    if ((!param.m_solver_robustifier_type.empty()) &&
        (param.m_solver_robustifier_num_iterations_coarse > 0) &&
        (param.m_solver_robustifier_threshold_fine >
         param.m_solver_robustifier_threshold_coarse))
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | if robustifier enabled, "
          "fine threshold must be lower than coarse threshold");
    }
    if (param.m_failure_ratio_inliers >= 1.f)
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | if maximum inliers ratio threshold to "
          "trigger the failure status of the aligner enabled, ratio must be "
          "between [0..1[");
    }
    if ((param.m_debug) && (param.m_dbg_ogl_interval < 0))
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | sleep interval "
          "between events must be equal or greater than 0");
    }
    if ((param.m_debug) &&
        (param.m_dbg_ogl_mode.compare("iterationend") != 0) &&
        (param.m_dbg_ogl_mode.compare("computeend") != 0))
    {
      throw std::runtime_error(
          "AlignerBase::assertInit | invalid OpenGL-based "
          "debug mode (iterationend, computeend)");
    }
  }  // void AlignerBase::assertInit()

  void init()
  {
    using SimpleCriteriaType = srrg2_solver::SimpleTerminationCriteria;
    using RelativeGradientChiCriteriaType =
        srrg2_solver::RelativeGradientChiTerminationCriteria;
    using PerturbationNormCriteriaType =
        srrg2_solver::PerturbationNormTerminationCriteria;

    using SimpleCriteriaTypePtr = std::shared_ptr<SimpleCriteriaType>;
    using RelativeGradientChiCriteriaTypePtr =
        std::shared_ptr<RelativeGradientChiCriteriaType>;
    using PerturbationNormCriteriaTypePtr =
        std::shared_ptr<PerturbationNormCriteriaType>;

    using RobustifierSaturatedType = srrg2_solver::RobustifierSaturated;
    using RobustifierCauchyType = srrg2_solver::RobustifierCauchy;
    using RobustifierClampType = srrg2_solver::RobustifierClamp;
    using RobustifierHuberType = srrg2_solver::RobustifierHuber;

    using RobustifierSaturatedTypePtr =
        std::shared_ptr<RobustifierSaturatedType>;
    using RobustifierCauchyTypePtr = std::shared_ptr<RobustifierCauchyType>;
    using RobustifierClampTypePtr = std::shared_ptr<RobustifierClampType>;
    using RobustifierHuberTypePtr = std::shared_ptr<RobustifierHuberType>;

    m_graph_.clear();
    m_solver_.param_actions.value().clear();
    m_solver_.param_robustifier_policies.value().clear();

    m_solver_.param_verbose.setValue(m_param_.m_solver_verbose);
    if (m_param_.m_solver_verbose)
    {
      m_solver_.param_actions.pushBack(
          std::make_shared<srrg2_solver::SolverVerboseLastAction>());
    }

    m_solver_.param_algorithm.setValue(std::make_shared<IterationAlgorithm>());
    dynamic_cast<IterationAlgorithm*>(m_solver_.param_algorithm.value().get())
        ->param_damping.setValue(m_param_.m_solver_damping);
    m_solver_.param_max_iterations.setValue({m_param_.m_solver_max_iterations});
    m_solver_.param_linear_solver.setValue(std::make_shared<LinearSolver>());

    if (m_param_.m_solver_criteria_type.empty())
    {
      m_solver_.param_termination_criteria.setValue(nullptr);
    }
    else if (m_param_.m_solver_criteria_type.compare("simple") == 0)
    {
      SimpleCriteriaTypePtr simple_criteria =
          std::make_shared<SimpleCriteriaType>();

      simple_criteria->param_epsilon.setValue(
          m_param_.m_solver_criteria_epsilon);
      simple_criteria->setName(m_param_.m_solver_criteria_type);

      m_solver_.param_termination_criteria.setValue(simple_criteria);
    }
    else if (m_param_.m_solver_criteria_type.compare("gradient") == 0)
    {
      RelativeGradientChiCriteriaTypePtr gradient_criteria =
          std::make_shared<RelativeGradientChiCriteriaType>();

      gradient_criteria->param_epsilon.setValue(
          m_param_.m_solver_criteria_epsilon);
      gradient_criteria->setName(m_param_.m_solver_criteria_type);

      m_solver_.param_termination_criteria.setValue(gradient_criteria);
    }
    else if (m_param_.m_solver_criteria_type.compare("stepnorm") == 0)
    {
      PerturbationNormCriteriaTypePtr norm_criteria =
          std::make_shared<PerturbationNormCriteriaType>();

      norm_criteria->param_epsilon.setValue(m_param_.m_solver_criteria_epsilon);
      norm_criteria->setName(m_param_.m_solver_criteria_type);

      m_solver_.param_termination_criteria.setValue(norm_criteria);
    }

    if (!m_param_.m_solver_robustifier_type.empty())
    {
      if (m_param_.m_solver_robustifier_num_iterations_coarse <= 0)
      {
        m_param_.m_solver_robustifier_threshold_coarse =
            m_param_.m_solver_robustifier_threshold_fine;
      }

      m_solver_.param_robustifier_policies.pushBack(
          std::make_shared<srrg2_solver::RobustifierPolicyByType>());

      if (m_param_.m_solver_robustifier_type.compare("saturated") == 0)
      {
        RobustifierSaturatedTypePtr saturated_robustifier =
            std::make_shared<RobustifierSaturatedType>();

        saturated_robustifier->setName(m_param_.m_solver_robustifier_type);
        saturated_robustifier->param_chi_threshold.setValue(
            m_param_.m_solver_robustifier_threshold_coarse);

        m_solver_.param_robustifier_policies.value(0)
            ->param_robustifier.setValue(saturated_robustifier);
      }
      else if (m_param_.m_solver_robustifier_type.compare("cauchy") == 0)
      {
        RobustifierCauchyTypePtr cauchy_robustifier =
            std::make_shared<RobustifierCauchyType>();

        cauchy_robustifier->setName(m_param_.m_solver_robustifier_type);
        cauchy_robustifier->param_chi_threshold.setValue(
            m_param_.m_solver_robustifier_threshold_coarse);

        m_solver_.param_robustifier_policies.value(0)
            ->param_robustifier.setValue(cauchy_robustifier);
      }
      else if (m_param_.m_solver_robustifier_type.compare("clamp") == 0)
      {
        RobustifierClampTypePtr clamp_robustifier =
            std::make_shared<RobustifierClampType>();

        clamp_robustifier->setName(m_param_.m_solver_robustifier_type);
        clamp_robustifier->param_chi_threshold.setValue(
            m_param_.m_solver_robustifier_threshold_coarse);

        m_solver_.param_robustifier_policies.value(0)
            ->param_robustifier.setValue(clamp_robustifier);
      }
      else if (m_param_.m_solver_robustifier_type.compare("huber") == 0)
      {
        RobustifierHuberTypePtr huber_robustifier =
            std::make_shared<RobustifierHuberType>();

        huber_robustifier->setName(m_param_.m_solver_robustifier_type);
        huber_robustifier->param_chi_threshold.setValue(
            m_param_.m_solver_robustifier_threshold_coarse);

        m_solver_.param_robustifier_policies.value(0)
            ->param_robustifier.setValue(huber_robustifier);
      }
    }

    if ((m_solver_.param_robustifier_policies.size() > 0) &&
        (m_param_.m_solver_robustifier_num_iterations_coarse > 0))
    {
      srrg2_solver::SolverDynamicRobustifierActionPtr
          dynamic_robustifier_action =
              std::make_shared<srrg2_solver::SolverDynamicRobustifierAction>();

      dynamic_robustifier_action->m_num_iterations_coarse =
          m_param_.m_solver_robustifier_num_iterations_coarse;

      dynamic_robustifier_action->m_robustifier_threshold_coarse =
          m_param_.m_solver_robustifier_threshold_coarse;

      dynamic_robustifier_action->m_robustifier_threshold_fine =
          m_param_.m_solver_robustifier_threshold_fine;

      dynamic_robustifier_action->m_robustifier =
          m_solver_.param_robustifier_policies.value(0)
              ->param_robustifier.value()
              .get();

      m_solver_.param_actions.pushBack(dynamic_robustifier_action);
    }

    m_solver_.setGraph(m_graph_);
  }  // void AlignerBase::init()

};  // class AlignerBase

/**
 * @brief aligner generic class
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class Aligner_ : public AlignerBase
{
 public:

  using typename AlignerBase::OGLAppType;
  using typename AlignerBase::OGLAppTypePtr;
  using typename AlignerBase::OGLCanvasType;
  using typename AlignerBase::OGLCanvasTypePtr;
  using typename AlignerBase::OGLViewerType;
  using typename AlignerBase::OGLViewerTypePtr;
  using typename AlignerBase::Param;

  using BaseType = AlignerBase;
  using ThisType = Aligner_<VariableSE2DistanceMapRightType_>;

  using BaseTypeParam = typename BaseType::Param;

  using VariableType = VariableSE2DistanceMapRightType_;
  using VariableTypePtr = std::shared_ptr<VariableType>;
  using EstimateType = typename VariableType::EstimateType;

  using MeasurementType = typename VariableType::MeasurementType;
  using MeasurementTypeElement = typename MeasurementType::element_type;

  using MeasurementOwnerType = typename VariableType::MeasurementOwnerType;
  using PointCloudTypePtr = typename MeasurementOwnerType::PointCloudTypePtr;
  using PointCloudType = typename MeasurementOwnerType::PointCloudType;
  using PointType = typename PointCloudType::PointType;
  using PointVectorType = typename PointType::VectorType;

  using PriorFactorType =
      srrg2_solver::SE2DistanceMapPriorErrorFactor_<VariableType>;
  using PriorFactorTypePtr = std::shared_ptr<PriorFactorType>;
  using PriorInformationMatrix =
      typename PriorFactorType::InformationMatrixType;

  using OGLPointCloudTargetType = srrg2_core::PointNormal3fVectorCloud;

  using Scalar = typename MeasurementOwnerType::DistanceMapType::Scalar;

  using EstimateTypeInfo = Eigen::Matrix<Scalar, VariableType::PerturbationDim,
                                         VariableType::PerturbationDim>;

 public:

  /**
   * @brief distance map parametrization
   */
  struct DistMapParam
  {
    Scalar
        m_dist_map_res;  //!< distance map resolution of the distance map (m/px)
    Scalar m_dist_map_size;  //!< distance map fixate the size (square width /
                             //!< 2) of the distance map (m) (-1 if you want to
                             //!< assume the maximum range in the point cloud
                             //!< used to setGoals)
    Scalar m_dist_map_max_dist;  //!< distance map maximum distance considered
                                 //!< to compute the distance map (m) (-1 if you
                                 //!< want to compute the whole distance map)

    bool m_solver_regularize_correspondences;  //!< enables regularization of
                                               //!< the information matrix when
                                               //!< more than one moving point
                                               //!< corresponds to the same
                                               //!< fixed point

    DistMapParam()
        : m_dist_map_res(0.01),
          m_dist_map_size(10.0),
          m_dist_map_max_dist(0.10),
          m_solver_regularize_correspondences(true)

    {
    }

    /**
     * @brief Construct a new DistMapParam object
     * @param[in] dist_map_res distance map resolution of distance map (m/px)
     * @param[in] dist_map_size distance map fixate the size (square width / 2)
     *                          of the distance map (m) (-1 if you want to
     *                          assume the maximum range in the point cloud used
     *                          to setGoals)
     * @param[in] dist_map_max_dist distance map maximum distance considered to
     *                              compute the distance map (m) (-1 if you want
     *                              to compute the whole distance map)
     * @param[in] solver_regularize_correspondences enables regularization of
     *                                              the information matrix when
     *                                              more than one moving point
     *                                              corresponds to the same
     *                                              fixed point
     */
    DistMapParam(Scalar dist_map_res, Scalar dist_map_size,
                 Scalar dist_map_max_dist,
                 bool solver_regularize_correspondences)
        : m_dist_map_res(dist_map_res),
          m_dist_map_size(dist_map_size),
          m_dist_map_max_dist(dist_map_max_dist),
          m_solver_regularize_correspondences(solver_regularize_correspondences)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, const DistMapParam& param)
    {
      os << "- distance map type                   : "
         << MeasurementTypeElement::DistanceMapType::getTypeName() << std::endl;
      os << "- distance map resolution       (m/px): " << param.m_dist_map_res
         << std::endl;
      os << "- distance map size                (m): ";
      if (param.m_dist_map_size <= 0)
      {
        os << "not enabled (size assuming the maximum range in the point cloud)"
           << std::endl;
      }
      else
      {
        os << param.m_dist_map_size << std::endl;
      }
      os << "- distance map maximum NN distance (m): ";
      if (param.m_dist_map_max_dist <= 0)
      {
        os << "not enabled (compute whole distance map based on its size)"
           << std::endl;
      }
      else
      {
        os << param.m_dist_map_max_dist << std::endl;
      }
      os << "- regularize *-to-1 correspondences   : "
         << (param.m_solver_regularize_correspondences ? "enabled"
                                                       : "not enable")
         << std::endl;

      return os;
    }  // friend std::ostream& operator<<(std::ostream& os, const StatsShort&
       // stats)
  };

 public:

  const Eigen::Vector4f OGLPointCloudTargetColorVar =
      srrg2_core::ColorPalette::color4fDarkBlue(0.75f);
  const Eigen::Vector4f OGLPointCloudMovingColorVar =
      srrg2_core::ColorPalette::color4fDarkRed(0.75f);
  const Eigen::Vector4f OGLPointCloudTargetColorPts =
      srrg2_core::ColorPalette::color4fBlack();

  const Eigen::Vector4f OGLVariableCovEllipsoidColor =
      srrg2_core::ColorPalette::color4fRed(0.75f);

 protected:

  VariableTypePtr m_variable_ =
      std::make_shared<VariableType>();  //!< variable to hold the
                                         //!< transformation between source
                                         //!< w.r.t. target measurement
  EstimateType m_estimate_ =
      EstimateType::Identity();  //!< SE(2) transformation estimation of the
                                 //!< source w.r.t. the target measurement

  PriorFactorTypePtr m_prior_ =
      std::make_shared<PriorFactorType>();  //!< SE(2) prior error factor for
                                            //!< the transformation estimation
                                            //!< of the source w.r.t. the target
                                            //!< measurement

  OGLPointCloudTargetType m_dbg_ogl_var_;  //!< OpenGL-based cached point cloud
                                           //!< for the target measurement

  DistMapParam m_dist_map_param_;  //!< distance map parametrization

  std::vector<int>
      m_moving_correspondences_;  //!< source correspondence for each point in
                                  //!< the moving point cloud
  std::unordered_map<int, int>
      m_source_correspondences_count_;  //!< associate source point idx to
                                        //!< correspondences count in the moving
                                        //!< point cloud

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct a new Aligner_<> object
   * @param[in] param aligner solver parametrization
   * @param[in] dist_map_param distance map parametrization
   */
  Aligner_(const Param& param = Param(),
           const DistMapParam& dist_map_param = DistMapParam())
      : AlignerBase(param), m_dist_map_param_(dist_map_param)
  {
    assertInit(m_dist_map_param_);
    init();
  }  // Aligner_::Aligner_(const Param& = Param())

  /**
   * @brief set the prior error factor for the aligner (e.g., based on the
   *        accumulated wheeled odometry estimation)
   * @param[in] prior SE(2) transformation estimate for the prior error factor
   * @param[in] prior_info_mat information matrix associated to the prior
   */
  void setPrior(const EstimateType& prior,
                const PriorInformationMatrix& prior_info_mat =
                    PriorInformationMatrix::Identity())
  {
    m_prior_->setEnabled(true);
    m_prior_->setMeasurement(prior);
    m_prior_->setInformationMatrix(prior_info_mat);
  }

  /**
   * @brief change the prior enabled status
   * @param[in] enable enabled status of the prior error factor
   */
  inline void setPriorEnabled(bool enable) { m_prior_->setEnabled(enable); }

  /**
   * @brief set source point cloud (defines the error factor)
   * @param[in] source moving point cloud
   * @param[in] sensor_in_robot SE(2) transformation of the sensor w.r.t. the
   *                            robot coordinate frame
   */
  virtual void setSource(const PointCloudTypePtr& source,
                         const EstimateType& sensor_in_robot) = 0;

  /**
   * @brief set source measurement (defines the error factor)
   * @param[in] source SE(2) variable representing the moving point cloud
   */
  virtual void setSource(VariableType* const source) = 0;

  /**
   * @brief set the source w.r.t. the target measurement's initial estimation
   *        for the solver's optimization process
   * @param[in] source_in_target SE(2) transformation initial estimation of the
   *                             source w.r.t. the target measurement
   */
  virtual void setSourceInTarget(const EstimateType& source_in_target)
  {
    m_estimate_ = source_in_target;
    m_variable_->setEstimate(m_estimate_);
  }

  /**
   * @brief set target point cloud (creates distance map)
   * @param[in] target fixed point cloud
   * @param[in] sensor_in_robot SE(2) transformation of the sensor w.r.t. the
   *                            robot coordinate frame
   */
  virtual void setTarget(const PointCloudTypePtr& target,
                         const EstimateType& sensor_in_robot) = 0;

  /**
   * @brief set target measurement (creates / represents the distance map)
   * @param[in] target SE(2) variable representing the fixed point cloud
   */
  virtual void setTarget(VariableType* const target) = 0;

  /**
   * @brief compute the alignment (solver::compute, computeStats, check failure,
   *        update internal status+estimate)
   */
  virtual void compute() override
  {
    this->m_solver_.compute();

    if (this->m_solver_.status() != srrg2_solver::Solver::SolverStatus::Success)
    {
      this->m_status_ = Status::Fail;
      return;
    }

    this->computeStats();

    if ((this->m_param_.m_failure_ratio_inliers >
         this->m_stats_.m_inliers_ratio) &&
        (this->m_param_.m_failure_ratio_inliers > 0.0f))
    {
      this->m_status_ = Status::NotEnoughInliersRatio;
      return;
    }

    if ((this->m_param_.m_failure_chi_inliers < this->m_stats_.m_chi_inliers) &&
        (this->m_param_.m_failure_chi_inliers > 0.0f))
    {
      this->m_status_ = Status::NotEnoughChiInliers;
      return;
    }

    this->m_status_ = Status::Success;
    this->m_estimate_ = this->m_variable_->estimate();
  }  // virtual void Aligner_::compute() override

  /**
   * @brief get approximate hessian matrix
   */
  EstimateTypeInfo getH() const
  {
    if (!this->m_solver_.H().numNonZeroBlocks())
    {
      return EstimateTypeInfo::Zero();
    }

    return this->m_solver_.H().blockAt(0, 0)->toMatrixXf();
  }

  /**
   * @brief update the canvas for the debug OpenGL-based visualization
   *        (only shows the target measurement)
   */
  virtual void updateDbgOPGLCanvas(bool update_canvas = true) override
  {
    if (!m_param_.m_debug)
    {
      throw std::runtime_error(
          "Aligner_::updateDbgOPGLCanvas | "
          "debug mode not enabled (only w/ dbg enabled the OGL viewer can be "
          "setup)");
    }
    if ((!m_dbg_ogl_qapp_) || (!m_dbg_ogl_viewer_) || (!m_dbg_ogl_canvas_))
    {
      throw std::runtime_error(
          "Aligner_::updateDbgOPGLCanvas | "
          "missing setup of the OpenGL-based debug visualization");
    }

    const srrg2_core::Isometry3_<Scalar> robot_in_world_3d =
        srrg2_core::geometry3d::get3dFrom2dPose(m_variable_->estimate());

    for (auto variable : m_graph_.variables())
    {
      const VariableType* v = static_cast<const VariableType*>(variable.second);

      const srrg2_core::Isometry3_<Scalar> sensor_in_robot_3d =
          srrg2_core::geometry3d::get3dFrom2dPose(v->getSensorInRobot());

      srrg2_core::Vector3f robot_to_sensor[2];
      robot_to_sensor[0].setZero();
      robot_to_sensor[1].x() = v->getSensorInRobot().translation().x();
      robot_to_sensor[1].y() = v->getSensorInRobot().translation().y();
      robot_to_sensor[1].z() = 0;

      m_dbg_ogl_canvas_->pushColor();
      m_dbg_ogl_canvas_->setColor(OGLPointCloudTargetColorVar);
      m_dbg_ogl_canvas_->putReferenceSystem(0.10);
      m_dbg_ogl_canvas_->popAttribute();

      m_dbg_ogl_canvas_->pushColor();
      m_dbg_ogl_canvas_->setColor(OGLPointCloudMovingColorVar);
      m_dbg_ogl_canvas_->pushMatrix();
      m_dbg_ogl_canvas_->multMatrix(robot_in_world_3d.matrix());
      m_dbg_ogl_canvas_->putReferenceSystem(0.05);
      m_dbg_ogl_canvas_->popMatrix();
      m_dbg_ogl_canvas_->popAttribute();

      m_dbg_ogl_canvas_->pushColor();
      m_dbg_ogl_canvas_->setColor(OGLPointCloudMovingColorVar);
      m_dbg_ogl_canvas_->pushMatrix();
      m_dbg_ogl_canvas_->multMatrix(robot_in_world_3d.matrix());
      m_dbg_ogl_canvas_->pushMatrix();
      m_dbg_ogl_canvas_->putLine(2, robot_to_sensor);
      m_dbg_ogl_canvas_->multMatrix(sensor_in_robot_3d.matrix());
      m_dbg_ogl_canvas_->putReferenceSystem(0.05);
      m_dbg_ogl_canvas_->popMatrix();
      m_dbg_ogl_canvas_->popMatrix();
      m_dbg_ogl_canvas_->popAttribute();

      if (!v->measurement())
      {
        continue;
      }
      if (!v->measurement()->getPointCloudPtr())
      {
        continue;
      }
      if (v->measurement()->getPointCloud()->size() < 1)
      {
        continue;
      }

      m_dbg_ogl_var_.resize(v->measurement()->getPointCloud()->size());

      for (size_t idx = 0; idx < v->measurement()->getPointCloud()->size();
           idx++)
      {
        m_dbg_ogl_var_[idx].coordinates().x() = v->measurement()
                                                    ->getPointCloud()
                                                    ->operator[](idx)
                                                    .coordinates()
                                                    .x();
        m_dbg_ogl_var_[idx].coordinates().y() = v->measurement()
                                                    ->getPointCloud()
                                                    ->operator[](idx)
                                                    .coordinates()
                                                    .y();
        m_dbg_ogl_var_[idx].coordinates().z() = 0;

        updateDbgOPGLCanvasShowNormals(idx, v);
      }

      m_dbg_ogl_canvas_->pushColor();
      m_dbg_ogl_canvas_->setColor(OGLPointCloudTargetColorPts);
      m_dbg_ogl_canvas_->pushMatrix();
      m_dbg_ogl_canvas_->multMatrix(sensor_in_robot_3d.matrix());
      m_dbg_ogl_canvas_->putPoints(m_dbg_ogl_var_);
      m_dbg_ogl_canvas_->popMatrix();
      m_dbg_ogl_canvas_->popAttribute();
    }

    EstimateTypeInfo info_mat = this->getH();

    if (!this->getH().isZero())
    {
      Eigen::SelfAdjointEigenSolver<EstimateTypeInfo> eigen_solver;

      eigen_solver.computeDirect(info_mat);

      const srrg2_core::Isometry3_<Scalar> robot_in_world_3d_translation =
          srrg2_core::geometry3d::get3dFrom2dPose(
              srrg2_core::geometry2d::v2t(srrg2_core::Vector3_<Scalar>(
                  m_variable_->estimate().translation().x(),
                  m_variable_->estimate().translation().y(), Scalar(0))));

      m_dbg_ogl_canvas_->pushColor();
      m_dbg_ogl_canvas_->setColor(OGLVariableCovEllipsoidColor);
      m_dbg_ogl_canvas_->pushMatrix();
      m_dbg_ogl_canvas_->multMatrix(robot_in_world_3d_translation.matrix());
      m_dbg_ogl_canvas_->putEllipsoid(srrg2_core::Vector3f(
          std::sqrt(1.f / eigen_solver.eigenvalues().real().x()) * 2.f,
          std::sqrt(1.f / eigen_solver.eigenvalues().real().y()) * 2.f, 0.05f));
      m_dbg_ogl_canvas_->popMatrix();
      m_dbg_ogl_canvas_->popAttribute();

      m_dbg_ogl_viewer_->lookAtPosition2D(
          m_dbg_ogl_canvas_name_, m_variable_->estimate().translation().x(),
          m_variable_->estimate().translation().y());
    }

    /**
     * @note the variable / estimate's information matrix only has a relative
     *       meaning in our case, given that we did not modelled the measurement
     *       noise of each point in the solver (current, identity matrix or, in
     *       case of the many-to-one correspondences regularization, weighted
     *       accordingly)...
     *       this debug visualization is only to introspect if the uncertainty
     *       along a certain dimension becomes too large or not!
     */

    if (update_canvas)
    {
      m_dbg_ogl_canvas_->flush();
    }
  }  // virtual void Aligner_::updateDbgOPGLCanvas()

  /**
   * @brief set new parametrization of the aligner solver and distance map
   *        configuration
   * @param[in] param aligner base parametrization
   * @param[in] dist_map_param distance map parametrization
   */
  void setDistMapParam(const Param& param, const DistMapParam& dist_map_param)
  {
    assertInit(dist_map_param);
    this->setParam(param);
    m_dist_map_param_ = dist_map_param;
    init();
  }

  /**
   * @brief set new distance map parametrization
   *        (does not need to update the whole configuration)
   * @param[in] dist_map_res resolution of the distance map (m/px)
   * @param[in] dist_map_size fixate the size (square width / 2)
   *                          of the distance map (m) (-1 if you want to
   *                          assume the maximum range in the point cloud
   *                          used to setGoals)
   * @param[in] dist_map_max_dist maximum distance considered to compute the
   *                              distance map (m) (-1 if you want to compute
   *                              the whole distance map)
   */
  inline void setDistMapParam(Scalar dist_map_res, Scalar dist_map_size,
                              Scalar dist_map_max_dist)
  {
    if (dist_map_res <= 0)
    {
      throw std::runtime_error(
          "Aligner_::setDistMapParam | distance map resolution must be greater "
          "than 0 (m)");
    }

    m_dist_map_param_.m_dist_map_res = dist_map_res;
    m_dist_map_param_.m_dist_map_size = dist_map_size;
    m_dist_map_param_.m_dist_map_max_dist = dist_map_max_dist;
  }

 public:

  inline VariableType* getVariable() { return m_variable_.get(); }
  virtual inline const EstimateType& getSourceInTarget() const
  {
    return m_estimate_;
  }
  inline const DistMapParam& getDistMapParam() const
  {
    return m_dist_map_param_;
  }

 protected:

  virtual void initCorrespondencesRegularizer() = 0;
  virtual void updateCorrespondencesRegularizer() = 0;

 private:

  static void assertInit(const DistMapParam& param)
  {
    if (param.m_dist_map_res <= 0)
    {
      throw std::runtime_error(
          "Aligner_::assertInit | distance map resolution must be greater than "
          "0 (m)");
    }
  }

  void init()
  {
    m_prior_->setEnabled(false);

    m_variable_->setGraphId(0);
    m_variable_->setEstimate(EstimateType::Identity());
    m_variable_->setMeasurement(std::make_shared<MeasurementTypeElement>());

    m_prior_->setVariableId(0, 0);
    m_prior_->setInformationMatrix(PriorInformationMatrix::Identity());

    this->m_graph_.addVariable(m_variable_);
    this->m_graph_.addFactor(m_prior_);

    if (m_dist_map_param_.m_solver_regularize_correspondences)
    {
      srrg2_solver::SolverGenericAtComputeStartActionPtr
          init_regularizer_action = std::make_shared<
              srrg2_solver::SolverGenericAtComputeStartAction>();

      srrg2_solver::SolverGenericAtIterationStartActionPtr
          update_regularizer_action = std::make_shared<
              srrg2_solver::SolverGenericAtIterationStartAction>();

      init_regularizer_action->m_action =
          std::bind(&ThisType::initCorrespondencesRegularizer, this);
      update_regularizer_action->m_action =
          std::bind(&ThisType::updateCorrespondencesRegularizer, this);

      this->m_solver_.param_actions.pushBack(init_regularizer_action);
      this->m_solver_.param_actions.pushBack(update_regularizer_action);
    }
  }

  template <typename T = PointType,
            std::enable_if_t<std::is_same<T, pcl::PointNormal2_<Scalar>>::value,
                             bool> = 0>
  inline void updateDbgOPGLCanvasShowNormals(size_t idx, const VariableType* v)
  {
    m_dbg_ogl_var_[idx].normal().x() =
        v->measurement()->getPointCloud()->operator[](idx).normal().x();
    m_dbg_ogl_var_[idx].normal().y() =
        v->measurement()->getPointCloud()->operator[](idx).normal().y();
    m_dbg_ogl_var_[idx].normal().z() = 0;
  }

  template <typename T = PointType,
            std::enable_if_t<
                !std::is_same<T, pcl::PointNormal2_<Scalar>>::value, bool> = 0>
  inline void updateDbgOPGLCanvasShowNormals(size_t, const VariableType*)
  {
  }

};  // class Aligner_<VariableSE2DistanceMapRightType_> : public AlignerBase

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
