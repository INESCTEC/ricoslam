#pragma once

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

/**
 * @brief variadic recursive template to get the type of a field pack item
 * @tparam FieldPackType_ field pack type (whole type)
 * @tparam idx item index in the field pack tupple
 */
template <typename FieldPackType_, int idx>
struct FieldPackItemTypeAt_
{
 public:

  using ThisType = FieldPackItemTypeAt_<FieldPackType_, idx>;
  using FieldPackItemType =
      typename FieldPackItemTypeAt_<typename FieldPackType_::FieldPackRestType,
                                    idx - 1>::FieldPackItemType;

};  // struct FieldPackItemTypeAt_<FieldPackType_, idx>

/**
 * @brief variadic recursive template to get the type of a field pack item
 *        (base class)
 * @tparam FieldPackType_ field pack type (whole type)
 */
template <typename FieldPackType_>
struct FieldPackItemTypeAt_<FieldPackType_, 0>
{
 public:

  using ThisType = FieldPackItemTypeAt_<FieldPackType_, 0>;
  using FieldPackItemType = typename FieldPackType_::FieldPackItemType;

};  // struct FieldPackItemTypeAt_<FieldPackType_, 0>

/**
 * @brief variadic recursive template to get a specific fied from a field pack
 *        class (getter)
 * @tparam FieldPackType_ field pack type (whole type)
 * @tparam idx item index in the field pack tupple
 */
template <typename FieldPackType_, int idx>
struct FieldPackItemFieldAt_
{
 public:

  using ThisType = FieldPackItemFieldAt_<FieldPackType_, idx>;

  using FieldPackType = FieldPackType_;
  using FieldPackItemType =
      typename FieldPackItemTypeAt_<FieldPackType, idx>::FieldPackItemType;

 public:

  static inline FieldPackItemType& field(FieldPackType& pack)
  {
    return FieldPackItemFieldAt_<typename FieldPackType_::FieldPackRestType,
                                 idx - 1>::field(pack);
  }

  static inline const FieldPackItemType& field(const FieldPackType& pack)
  {
    return FieldPackItemFieldAt_<typename FieldPackType_::FieldPackRestType,
                                 idx - 1>::field(pack);
  }

};  // struct FieldPackItemFieldAt_<FieldPackType_, idx>;

/**
 * @brief variadic recursive template to get a specific fied from a field pack
 *        class (getter) (base class)
 * @tparam FieldPackType_ field pack type (whole type)
 */
template <typename FieldPackType_>
struct FieldPackItemFieldAt_<FieldPackType_, 0>
{
 public:

  using ThisType = FieldPackItemFieldAt_<FieldPackType_, 0>;

  using FieldPackType = FieldPackType_;
  using FieldPackItemType =
      typename FieldPackItemTypeAt_<FieldPackType, 0>::FieldPackItemType;

 public:

  static inline FieldPackItemType& field(FieldPackType& pack)
  {
    return pack.m_field;
  }

  static inline const FieldPackItemType& field(const FieldPackType& pack)
  {
    return pack.m_field;
  }

};  // struct FieldPackItemFieldAt_<FieldPackType_, 0>

/**
 * @brief field pack tupple template definition (variadic recursive template)
 * @tparam FieldPackItemType_ field pack 'A SINGLE' item type
 * @tparam FieldPackRestItemType_ field pack 'REST' item types (variadic)
 */
template <typename FieldPackItemType_, typename... FieldPackRestItemType_>
struct FieldPack_ : public FieldPack_<FieldPackRestItemType_...>
{
 public:

  using ThisType = FieldPack_<FieldPackItemType_, FieldPackRestItemType_...>;

  using FieldPackItemType = FieldPackItemType_;
  using FieldPackRestType = FieldPack_<FieldPackRestItemType_...>;

  template <int idx>
  using FieldPackItemTypeAt =
      typename FieldPackItemTypeAt_<ThisType, idx>::FieldPackItemType;

 public:

  static constexpr int kNumFields = 1 + sizeof...(FieldPackRestItemType_);

  FieldPackItemType
      m_field;  //!< field item in tupple pack (variadic recursive definition)

 public:

  FieldPack_() = default;
  FieldPack_(const FieldPackItemType& field,
             const FieldPackRestItemType_&... rest)
      : FieldPack_<FieldPackRestItemType_...>(rest...), m_field(field)
  {
  }

  /**
   * @brief method here for generalization purposes, to have the same API as the
   *        FieldPackDerived_ data structure
   * @tparam idx item index in the field pack tupple
   * @return constexpr int item index in the field pack structure composed by
   *                       the base field pack tupple
   */
  template <int idx>
  static constexpr int unrolledIdx()
  {
    return idx;
  }

  template <int idx>
  inline FieldPackItemTypeAt<idx>& field()
  {
    return FieldPackItemFieldAt_<ThisType, idx>::field(*this);
  }

  template <int idx>
  inline const FieldPackItemTypeAt<idx>& field() const
  {
    return FieldPackItemFieldAt_<ThisType, idx>::field(*this);
  }

};  // struct FieldPack_<FieldPackItemType_, FieldPackRestItemType_...> : public
    // FieldPack_<FieldPackRestItemType_...>

/**
 * @brief field pack tupple template definition (base class template)
 * @tparam FieldPackItemType_ field pack 'A SINGLE' item type
 */
template <typename FieldPackItemType_>
struct FieldPack_<FieldPackItemType_>
{
 public:

  using ThisType = FieldPack_<FieldPackItemType_>;

  using FieldPackItemType = FieldPackItemType_;

  template <int idx>
  using FieldPackItemTypeAt =
      typename FieldPackItemTypeAt_<ThisType, idx>::FieldPackItemType;

 public:

  static constexpr int kNumFields = 1;

  FieldPackItemType
      m_field;  //!< field item in tupple pack (base class for recursion)

 public:

  FieldPack_() = default;
  FieldPack_(const FieldPackItemType& field) : m_field(field) {}

  template <int idx>
  static constexpr int unrolledIdx()
  {
    return idx;
  }

  template <int idx>
  inline FieldPackItemTypeAt<idx>& field()
  {
    return FieldPackItemFieldAt_<ThisType, idx>::field(*this);
  }

  template <int idx>
  inline const FieldPackItemTypeAt<idx>& field() const
  {
    return FieldPackItemFieldAt_<ThisType, idx>::field(*this);
  }

};  // struct FieldPack_<FieldPackItemType_>

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
