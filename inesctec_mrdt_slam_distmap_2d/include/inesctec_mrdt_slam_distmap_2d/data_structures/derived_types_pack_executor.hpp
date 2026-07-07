#pragma once

#include <type_traits>
#include <utility>

namespace inesctec_mrdt_slam_distmap_2d
{
namespace data_structures
{

/**
 * @brief variadic recursive template to execute a function of an unknown object
 *        type based on its parent base class pointer
 *
 * @note well, if you are using this structure, possibly you have either a bad
 *       code architecture or you are just a lazzy developer... still, useful I
 *       think, specially for the distance map-based variables, due to their
 *       extensive templification relative to VariableSE2Base...
 *
 * @tparam DerivedTypesPackExecutorItemType_ current single derived type
 * @tparam DerivedTypesPackExecutorRestItemType_ remaining types (variadic)
 */
template <typename DerivedTypesPackExecutorItemType_,
          typename... DerivedTypesPackExecutorRestItemType_>
struct DerivedTypesPackExecutor_
    : public DerivedTypesPackExecutor_<DerivedTypesPackExecutorRestItemType_...>
{
 public:

  using ThisType =
      DerivedTypesPackExecutor_<DerivedTypesPackExecutorItemType_,
                                DerivedTypesPackExecutorRestItemType_...>;

  using DerivedTypesPackExecutorItemType = DerivedTypesPackExecutorItemType_;
  using DerivedTypesPackExecutorRestType =
      DerivedTypesPackExecutor_<DerivedTypesPackExecutorRestItemType_...>;

 public:

  static constexpr int kNumTypes =
      1 + sizeof...(DerivedTypesPackExecutorRestItemType_);

 public:

  /**
   * @brief evaluates if the base pointer can be caster to the current single
   *        derived type
   * @tparam BaseType base parent class type
   * @param[in] ptr base parent class pointer
   * @return true base pointer can be casted to the current single derived type
   * @return false otherwise
   */
  template <typename BaseType>
  static bool testCurrentType(BaseType* ptr)
  {
    return dynamic_cast<DerivedTypesPackExecutorItemType*>(ptr) != nullptr;
  }

  /**
   * @brief recursive find matching type implementation (index are from right to
   *        left template definition)
   * @tparam BaseType base parent class type
   * @param[in] ptr base parent class pointer
   * @return int index in the types parameter pack (right to left indexation)
   */
  template <typename BaseType>
  static int findMatchingType(BaseType* ptr)
  {
    if (testCurrentType(ptr))
    {
      return kNumTypes - 1;
    }
    return DerivedTypesPackExecutorRestType::findMatchingType(ptr);
  }

  /**
   * @brief recursive function execution based on the find matching type
   * @tparam BaseType base parent class type
   * @tparam FuncType executor function template (e.g., compiler derived)
   * @param[in] ptr base parent class pointer
   * @param[in] func executor function object (e.g., lambda)
   * @return true executor function was called
   * @return false otherwise
   */
  template <typename BaseType, typename FuncType>
  static bool executeWithMatchingType(BaseType* ptr, FuncType&& func)
  {
    if (auto casted = dynamic_cast<DerivedTypesPackExecutorItemType*>(ptr))
    {
      func(casted);
      return true;
    }
    return DerivedTypesPackExecutorRestType::executeWithMatchingType(
        ptr, std::forward<FuncType>(func));
  }
};

/**
 * @brief variadic recursive template to execute a function of an unknown object
 *        type based on its parent base class pointer
 *
 * @note well, if you are using this structure, possibly you have either a bad
 *       code architecture or you are just a lazzy developer... still, useful I
 *       think, specially for the distance map-based variables, due to their
 *       extensive templification relative to VariableSE2Base...
 *
 * @tparam DerivedTypesPackExecutorItemType_ current single derived type
 */
template <typename DerivedTypesPackExecutorItemType_>
struct DerivedTypesPackExecutor_<DerivedTypesPackExecutorItemType_>
{
 public:

  using ThisType = DerivedTypesPackExecutor_<DerivedTypesPackExecutorItemType_>;
  using DerivedTypesPackExecutorItemType = DerivedTypesPackExecutorItemType_;

 public:

  static constexpr int kNumTypes = 1;

 public:

  /**
   * @brief evaluates if the base pointer can be caster to the current single
   *        derived type
   * @tparam BaseType base parent class type
   * @param[in] ptr base parent class pointer
   * @return true base pointer can be casted to the current single derived type
   * @return false otherwise
   */
  template <typename BaseType>
  static bool testCurrentType(BaseType* ptr)
  {
    return dynamic_cast<DerivedTypesPackExecutorItemType*>(ptr) != nullptr;
  }

  /**
   * @brief base find matching type implementation (index are from right to
   *        left template definition)
   * @tparam BaseType base parent class type
   * @param[in] ptr base parent class pointer
   * @return int 0 index if valid, -1 otherwise
   */
  template <typename BaseType>
  static int findMatchingType(BaseType* ptr)
  {
    return testCurrentType(ptr) ? 0 : -1;
  }

  /**
   * @brief base function execution based on the find matching type
   * @tparam BaseType base parent class type
   * @tparam FuncType executor function template (e.g., compiler derived)
   * @param[in] ptr base parent class pointer
   * @param[in] func executor function object (e.g., lambda)
   * @return true executor function was called
   * @return false otherwise
   */
  template <typename BaseType, typename FuncType>
  static bool executeWithMatchingType(BaseType* ptr, FuncType&& func)
  {
    if (auto casted = dynamic_cast<DerivedTypesPackExecutorItemType*>(ptr))
    {
      func(casted);
      return true;
    }
    return false;
  }
};

}  // namespace data_structures
}  // namespace inesctec_mrdt_slam_distmap_2d
