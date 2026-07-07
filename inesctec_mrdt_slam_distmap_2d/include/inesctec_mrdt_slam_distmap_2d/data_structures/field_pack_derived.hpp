#pragma once

#include "inesctec_mrdt_slam_distmap_2d/data_structures/field_pack.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

/**
 * @brief variadic recursive template to define a field pack derived from a base
 *        field pack class (variadic recursive template)
 * @note the indexation without the unrolled index method is first the derived
 *       field items, and only then, the field items from the base field pack
 *       (that is why the need of unrolled...)
 * @tparam FieldPackBaseType_ base field pack type
 * @tparam FieldPackItemType_ field pack 'A SINGLE' item type
 * @tparam FieldPackRestItemType_ field pack 'REST' item types (variadic)
 */
template <typename FieldPackBaseType_, typename FieldPackItemType_,
          typename... FieldPackRestItemType_>
struct FieldPackDerived_
    : public FieldPackDerived_<FieldPackBaseType_, FieldPackRestItemType_...>
{
 public:

  using ThisType = FieldPackDerived_<FieldPackBaseType_, FieldPackItemType_,
                                     FieldPackRestItemType_...>;

  using FieldPackBaseType = FieldPackBaseType_;
  using FieldPackItemType = FieldPackItemType_;
  using FieldPackRestType =
      FieldPackDerived_<FieldPackBaseType_, FieldPackRestItemType_...>;

 public:

  static constexpr int kNumFields =
      FieldPackBaseType::kNumFields + 1 + sizeof...(FieldPackRestItemType_);

  static constexpr int kNumOuterFields = 1 + sizeof...(FieldPackRestItemType_);

  FieldPackItemType
      m_field;  //!< field item in tupple pack (variadic recursive definition)

 public:

  FieldPackDerived_() = default;
  FieldPackDerived_(const FieldPackBaseType& base,
                    const FieldPackItemType& field,
                    const FieldPackRestItemType_&... rest)
      : FieldPackRestType(base, rest...), m_field(field)
  {
  }

  /**
   * @brief method required to make the implementation agnostic to the way of
   *        variadic derived field pack type defines its internal field indexes
   *        (instead of putting the added fields of the derived type at the end,
   *        these fields are put at the beginning of the definition)
   * @tparam idx item index in the field pack tupple
   * @return constexpr int item unrolled index agnostic to the internal index
   *                       implementation
   */
  template <int idx>
  static constexpr int unrolledIdx()
  {
    return (idx < FieldPackBaseType::kNumFields
                ? kNumOuterFields +
                      FieldPackBaseType::template unrolledIdx<idx>()
                : idx - FieldPackBaseType::kNumFields);
  }

  template <int idx>
  using FieldPackItemTypeAt =
      typename FieldPackItemTypeAt_<ThisType,
                                    unrolledIdx<idx>()>::FieldPackItemType;

  template <int idx>
  inline FieldPackItemTypeAt<idx>& field()
  {
    return FieldPackItemFieldAt_<ThisType, unrolledIdx<idx>()>::field(*this);
  }

  template <int idx>
  inline const FieldPackItemTypeAt<idx>& field() const
  {
    return FieldPackItemFieldAt_<ThisType, unrolledIdx<idx>()>::field(*this);
  }

};  // struct FieldPackDerived_<FieldPackBaseType_, FieldPackItemType_,
    // FieldPackRestItemType_...> : public FieldPackDerived_<FieldPackBaseType_,
    // FieldPackRestItemType_...>

/**
 * @brief variadic recursive template to define a field pack derived from a base
 *        field pack class (base class template)
 * @note the indexation without the unrolled index method is first the derived
 *       field items, and only then, the field items from the base field pack
 *       (that is why the need of unrolled...)
 * @tparam FieldPackBaseType_ base field pack type
 * @tparam FieldPackItemType_ field pack 'A SINGLE' item type
 */
template <typename FieldPackBaseType_, typename FieldPackItemType_>
struct FieldPackDerived_<FieldPackBaseType_, FieldPackItemType_>
    : public FieldPackBaseType_
{
 public:

  using ThisType = FieldPackDerived_<FieldPackBaseType_, FieldPackItemType_>;

  using FieldPackBaseType = FieldPackBaseType_;
  using FieldPackItemType = FieldPackItemType_;
  using FieldPackRestType = FieldPackBaseType_;

 public:

  static constexpr int kNumFields = FieldPackBaseType::kNumFields + 1;

  static constexpr int kNumOuterFields = 1;

  FieldPackItemType
      m_field;  //!< field item in tupple pack (variadic recursive definition)

 public:

  FieldPackDerived_<FieldPackBaseType_, FieldPackItemType_>() = default;
  FieldPackDerived_<FieldPackBaseType_, FieldPackItemType_>(
      const FieldPackBaseType& base, const FieldPackItemType& field)
      : FieldPackBaseType_(base), m_field(field)
  {
  }

  /**
   * @brief method required to make the implementation agnostic to the way of
   *        variadic derived field pack type defines its internal field indexes
   *        (instead of putting the added fields of the derived type at the end,
   *        these fields are put at the beginning of the definition)
   * @tparam idx item index in the field pack tupple
   * @return constexpr int item unrolled index agnostic to the internal index
   *                       implementation
   */
  template <int idx>
  static constexpr int unrolledIdx()
  {
    return (idx < FieldPackBaseType::kNumFields
                ? kNumOuterFields +
                      FieldPackBaseType::template unrolledIdx<idx>()
                : idx - FieldPackBaseType::kNumFields);
  }

  template <int idx>
  using FieldPackItemTypeAt =
      typename FieldPackItemTypeAt_<ThisType,
                                    unrolledIdx<idx>()>::FieldPackItemType;

  template <int idx>
  inline FieldPackItemTypeAt<idx>& field()
  {
    return FieldPackItemFieldAt_<ThisType, unrolledIdx<idx>()>::field(*this);
  }

  template <int idx>
  inline const FieldPackItemTypeAt<idx>& field() const
  {
    return FieldPackItemFieldAt_<ThisType, unrolledIdx<idx>()>::field(*this);
  }

};  // struct FieldPackDerived_<FieldPackBaseType_, FieldPackItemType_> : public
    // FieldPackBaseType_

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
