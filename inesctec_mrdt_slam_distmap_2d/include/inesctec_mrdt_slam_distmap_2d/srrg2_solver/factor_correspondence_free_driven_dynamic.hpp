#pragma once

#include <memory>
#include <vector>

// Eigen
#include <Eigen/StdVector>

// SRRG Software
#include <srrg2_core/srrg_data_structures/iterator_interface.h>
#include <srrg2_solver/solver_core/factor_base.h>

namespace srrg2_solver
{

/**
 * @brief outer factor correspondence free driven to be compatible with the SRRG
 *        solver when adding a factor that is indeed a set of points related to
 *        the same variable (e.g., point cloud)
 *
 * in this case, correspondence free due to the distance map formulation giving
 * indirectly the association between moving point to fixed one...
 *
 * @note this factor differs from FactorCorrespondenceFreeDriven_ by having a
 *       shared pointer to the point cloud, instead of leaving the handling
 *       responsability of the cloud to external classes / structures!
 *
 * @tparam FactorBaseType_ factor singular instance base type
 * @tparam MovingContainerType_ moving container type
 */
template <typename FactorBaseType_, typename MovingContainerType_>
class FactorCorrespondenceFreeDrivenDynamic_ : public FactorBaseType_
{
 public:

  using ThisType = FactorCorrespondenceFreeDrivenDynamic_<FactorBaseType_,
                                                          MovingContainerType_>;
  using BaseType = FactorBaseType_;

  using MovingContainerType = MovingContainerType_;
  using MovingContainerTypePtr = std::shared_ptr<MovingContainerType>;
  using MovingItemType = typename MovingContainerType::value_type;
  using MovingIteratorType = typename MovingContainerType::const_iterator;

  using InformationMatrixType = typename BaseType::InformationMatrixType;
  using InformationMatrixVectorType =
      std::vector<InformationMatrixType,
                  Eigen::aligned_allocator<InformationMatrixType>>;

 protected:

  MovingContainerTypePtr m_moving_ =
      nullptr;  //!< moving set of correspondence free factors
  const InformationMatrixVectorType* m_info_matrix_vector_ =
      nullptr;  //!< information matrix vector for the moving set of factors

  MovingIteratorType
      m_moving_it_;  //!< internal moving iterator over the original set of
                     //!< correspondence free factors

  srrg2_solver::FactorBase* m_factor_ = nullptr;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief set the information matrix vector (each element in the vector must
   *        be a direct correspondence to the moving container, i.e., same
   *        indexation / correspondences)
   * @param[in] info information matrix vector for the moving set of factors
   */
  void setInformationMatrixVector(const InformationMatrixVectorType& info)
  {
    m_info_matrix_vector_ = &info;
  }

  /**
   * @brief set the moving set of correspondence free factors
   * @param[in] moving moving set of correspondence free factors
   */
  void setMoving(const MovingContainerType& moving)
  {
    m_moving_ = std::make_shared<MovingContainerType>(moving);
    setBegin();
  }

  /**
   * @brief set the moving set of correspondence free factors
   * @param[in] moving moving set of correspondence free factors
   */
  void setMoving(const MovingContainerTypePtr& moving)
  {
    m_moving_ = moving;
    setBegin();
  }

  void setBegin() override { m_moving_it_ = m_moving_->begin(); }
  bool isEnd() override { return (m_moving_it_ == m_moving_->end()); }

  inline const MovingContainerType* getMoving() const
  {
    return m_moving_.get();
  }
  inline const MovingContainerTypePtr& getMovingPtr() const
  {
    return m_moving_;
  }
  const MovingIteratorType& getMovingIt() const { return m_moving_it_; }

  size_t size() override { return m_moving_->size(); }

  /**
   * @brief get the factor in the SRRG's base abstraction factor type and
   *        execute BaseType::setMoving and BaseType::setInformationMatrix
   *        of the current individual / single correspondence free factor
   *        (wo/ updating the internal iterator)
   * @return srrg2_solver::FactorBase*& factor
   */
  srrg2_solver::FactorBase*& get() override
  {
    assert(((!m_info_matrix_vector_) ||
            (m_info_matrix_vector_->size() == m_moving_->size())) &&
           "FactorCorrespondenceFreeDrivenDynamic_::get | "
           "information matrix vector size mismatch with fixed data");

    if (m_moving_it_ == m_moving_->end())
    {
      m_factor_ = nullptr;
      return m_factor_;
    }

    const size_t moving_idx =
        std::distance<MovingIteratorType>(m_moving_->begin(), m_moving_it_);

    if (moving_idx < 0)
    {
      m_factor_ = nullptr;
      return m_factor_;
    }

    BaseType::setMoving(m_moving_->at(moving_idx));

    if (m_info_matrix_vector_)
    {
      BaseType::setInformationMatrix(m_info_matrix_vector_->at(moving_idx));
    }

    m_factor_ = this;
    return m_factor_;
  }

  /**
   * @brief update the internal moving iterator over the original set of
   *        correspondence free factors
   * @return srrg2_core::IteratorInterface_<srrg2_solver::FactorBase*>& next
   * factor
   */
  srrg2_core::IteratorInterface_<srrg2_solver::FactorBase*>& next() override
  {
    ++m_moving_it_;
    return *this;
  }

  /**
   * @brief execute the BaseType::setMoving and BaseType::setInformationMatrix
   *        methods of the current individual / single correspondence free
   *        factor and update the internal iterator automatically
   * @param[in] moving factor
   * @return true internal moving iterator HAS NOT reached the end of container
   * @return false otherwise
   */
  virtual bool getNext(srrg2_solver::FactorBase*& moving)
  {
    assert(((!m_info_matrix_vector_) ||
            (m_info_matrix_vector_->size() == m_moving_->size())) &&
           "FactorCorrespondenceFreeDrivenDynamic_::getNext | "
           "information matrix vector size mismatch with fixed scene");

    moving = nullptr;

    if (m_moving_it_ == m_moving_->end())
    {
      return false;
    }

    const size_t moving_idx =
        std::distance<MovingIteratorType>(m_moving_->begin(), m_moving_it_);

    if (moving_idx >= 0)
    {
      BaseType::setMoving(m_moving_->at(moving_idx));

      if (m_info_matrix_vector_)
      {
        BaseType::setInformationMatrix(m_info_matrix_vector_->at(moving_idx));
      }

      moving = this;
    }

    ++m_moving_it_;

    return true;
  }

};  // class FactorCorrespondenceFreeDrivenDynamic_<FactorBaseType_,
    // MovingContainerType_> : public FactorBaseType_

}  // namespace srrg2_solver
