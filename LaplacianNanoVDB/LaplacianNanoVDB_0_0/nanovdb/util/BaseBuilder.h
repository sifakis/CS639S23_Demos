// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

/*!
    \file BaseBuilder.h

    \author Ken Museth

    \date June 26, 2020

    \brief Generates a NanoVDB grid from any volume or function.

    \note This is only intended as a simple tool to generate nanovdb grids without
          any dependency on openvdb.
*/

#ifndef NANOVDB_BASEBUILDER_H_HAS_BEEN_INCLUDED
#define NANOVDB_BASEBUILDER_H_HAS_BEEN_INCLUDED

#include <map>
#include <limits>
#include <sstream> // for stringstream
#include <vector>
#include <cstring> // for memcpy

#include <nanovdb/NanoVDB.h>

namespace nanovdb {

namespace build {

template<typename ChildT>
struct RootNode
{
    using ValueType = typename ChildT::ValueType;
    using ChildType = ChildT;
    static constexpr uint32_t LEVEL = 1 + ChildT::LEVEL; // level 0 = leaf
    struct Tile {
        Tile(ChildT* c = nullptr) : child(c) {}
        Tile(const ValueType& v, bool s) : child(nullptr), value(v), state(s) {}
        ChildT*   child;
        ValueType value;
        bool      state;
    };
    using MapT = std::map<Coord, Tile>;
    MapT      mTable;
    ValueType mBackground;

    RootNode(const ValueType& background) : mBackground(background) {}
    RootNode(const RootNode&) = delete; // disallow copy-construction
    RootNode(RootNode&&) = default; // allow move construction
    RootNode& operator=(const RootNode&) = delete; // disallow copy assignment
    RootNode& operator=(RootNode&&) = default; // allow move assignment

    ~RootNode() { this->clear(); }

    bool empty() const { return mTable.empty(); }

    void clear()
    {
        for (auto iter = mTable.begin(); iter != mTable.end(); ++iter)
            delete iter->second.child;
        mTable.clear();
    }

    static Coord CoordToKey(const Coord& ijk) { return ijk & ~ChildT::MASK; }

    template<typename AccT>
    bool isActiveAndCache(const Coord& ijk, AccT& acc) const
    {
        auto iter = mTable.find(CoordToKey(ijk));
        if (iter == mTable.end())
            return false;
        if (iter->second.child) {
            acc.insert(ijk, iter->second.child);
            return iter->second.child->isActiveAndCache(ijk, acc);
        }
        return iter->second.state;
    }

    ValueType getValue(const Coord& ijk) const
    {
        auto iter = mTable.find(CoordToKey(ijk));
        if (iter == mTable.end()) {
            return mBackground;
        } else if (iter->second.child) {
            return iter->second.child->getValue(ijk);
        } else {
            return iter->second.value;
        }
    }

    template<typename AccT>
    ValueType getValueAndCache(const Coord& ijk, AccT& acc) const
    {
        auto iter = mTable.find(CoordToKey(ijk));
        if (iter == mTable.end())
            return mBackground;
        if (iter->second.child) {
            acc.insert(ijk, iter->second.child);
            return iter->second.child->getValueAndCache(ijk, acc);
        }
        return iter->second.value;
    }

    template<typename AccT>
    void setValueAndCache(const Coord& ijk, const ValueType& value, AccT& acc)
    {
        ChildT*     child = nullptr;
        const Coord key = CoordToKey(ijk);
        auto        iter = mTable.find(key);
        if (iter == mTable.end()) {
            child = new ChildT(ijk, mBackground, false);
            mTable[key] = Tile(child);
        } else if (iter->second.child != nullptr) {
            child = iter->second.child;
        } else {
            child = new ChildT(ijk, iter->second.value, iter->second.state);
            iter->second.child = child;
        }
        NANOVDB_ASSERT(child);
        acc.insert(ijk, child);
        child->setValueAndCache(ijk, value, acc);
    }

    template<typename NodeT>
    uint32_t nodeCount() const
    {
        static_assert(is_same<ValueType, typename NodeT::ValueType>::value, "Root::getNodes: Invalid type");
        static_assert(NodeT::LEVEL < LEVEL, "Root::getNodes: LEVEL error");
        uint32_t sum = 0;
        for (auto iter = mTable.begin(); iter != mTable.end(); ++iter) {
            if (iter->second.child == nullptr)
                continue; // skip tiles
            if (is_same<NodeT, ChildT>::value) { //resolved at compile-time
                ++sum;
            } else {
                sum += iter->second.child->template nodeCount<NodeT>();
            }
        }
        return sum;
    }

    template<typename NodeT>
    void getNodes(std::vector<NodeT*>& array)
    {
        static_assert(is_same<ValueType, typename NodeT::ValueType>::value, "Root::getNodes: Invalid type");
        static_assert(NodeT::LEVEL < LEVEL, "Root::getNodes: LEVEL error");
        for (auto iter = mTable.begin(); iter != mTable.end(); ++iter) {
            if (iter->second.child == nullptr)
                continue;
            if (is_same<NodeT, ChildT>::value) { //resolved at compile-time
                array.push_back(reinterpret_cast<NodeT*>(iter->second.child));
            } else {
                iter->second.child->getNodes(array);
            }
        }
    }

    void addChild(ChildT*& child)
    {
        NANOVDB_ASSERT(child);
        const Coord key = CoordToKey(child->mOrigin);
        auto        iter = mTable.find(key);
        if (iter != mTable.end() && iter->second.child != nullptr) { // existing child node
            delete iter->second.child;
            iter->second.child = child;
        } else {
            mTable[key] = Tile(child);
        }
        child = nullptr;
    }

    template<typename NodeT>
    void addNode(NodeT*& node)
    {
        if (is_same<NodeT, ChildT>::value) { //resolved at compile-time
            this->addChild(reinterpret_cast<ChildT*&>(node));
        } else {
            ChildT*     child = nullptr;
            const Coord key = CoordToKey(node->mOrigin);
            auto        iter = mTable.find(key);
            if (iter == mTable.end()) {
                child = new ChildT(node->mOrigin, mBackground, false);
                mTable[key] = Tile(child);
            } else if (iter->second.child != nullptr) {
                child = iter->second.child;
            } else {
                child = new ChildT(node->mOrigin, iter->second.value, iter->second.state);
                iter->second.child = child;
            }
            child->addNode(node);
        }
    }

    template<typename T>
    typename std::enable_if<std::is_floating_point<T>::value>::type
    signedFloodFill(T outside);

    template<typename T>
    typename std::enable_if<!std::is_floating_point<T>::value>::type
    signedFloodFill(T) {} // no-op for none floating point values
}; // RootNode

//================================================================================================

template<typename ChildT>
template<typename T>
inline typename std::enable_if<std::is_floating_point<T>::value>::type
RootNode<ChildT>::signedFloodFill(T outside)
{
    std::map<Coord, ChildT*> nodeKeys;
    for (auto iter = mTable.begin(); iter != mTable.end(); ++iter) {
        if (iter->second.child == nullptr)
            continue;
        nodeKeys.insert(std::pair<Coord, ChildT*>(iter->first, iter->second.child));
    }

    // We employ a simple z-scanline algorithm that inserts inactive tiles with
    // the inside value if they are sandwiched between inside child nodes only!
    auto b = nodeKeys.begin(), e = nodeKeys.end();
    if (b == e)
        return;
    for (auto a = b++; b != e; ++a, ++b) {
        Coord d = b->first - a->first; // delta of neighboring coordinates
        if (d[0] != 0 || d[1] != 0 || d[2] == int(ChildT::DIM))
            continue; // not same z-scanline or neighbors
        const ValueType fill[] = {a->second->getLastValue(), b->second->getFirstValue()};
        if (!(fill[0] < 0) || !(fill[1] < 0))
            continue; // scanline isn't inside
        Coord c = a->first + Coord(0u, 0u, ChildT::DIM);
        for (; c[2] != b->first[2]; c[2] += ChildT::DIM) {
            const Coord key = RootNode<ChildT>::CoordToKey(c);
            mTable[key] = typename RootNode<ChildT>::Tile(-outside, false); // inactive tile
        }
    }
} // RootNode::signedFloodFill

//================================================================================================

template<typename ChildT>
struct InternalNode
{
    using ValueType = typename ChildT::ValueType;
    using BuildType = typename ChildT::BuildType;
    using ChildType = ChildT;
    static constexpr uint32_t LOG2DIM = ChildT::LOG2DIM + 1;
    static constexpr uint32_t TOTAL = LOG2DIM + ChildT::TOTAL; //dimension in index space
    static constexpr uint32_t DIM = 1u << TOTAL;
    static constexpr uint32_t SIZE = 1u << (3 * LOG2DIM); //number of tile values (or child pointers)
    static constexpr int32_t  MASK = DIM - 1;
    static constexpr uint32_t LEVEL = 1 + ChildT::LEVEL; // level 0 = leaf
    static constexpr uint64_t NUM_VALUES = uint64_t(1) << (3 * TOTAL); // total voxel count represented by this node
    using MaskT = Mask<LOG2DIM>;
    using NanoNodeT = typename NanoNode<BuildType, LEVEL>::Type;

    struct Tile {
        Tile(ChildT* c = nullptr) : child(c) {}
        union{
            ChildT*   child;
            ValueType value;
        };
    };
    Coord      mOrigin;
    MaskT      mValueMask;
    MaskT      mChildMask;
    Tile       mTable[SIZE];

    union {
        NanoNodeT *mDstNode;
        uint64_t   mDstOffset;
    };

    InternalNode(const Coord& origin, const ValueType& value, bool state)
        : mOrigin(origin & ~MASK)
        , mValueMask(state)
        , mChildMask()
        , mDstOffset(0)
    {
        for (uint32_t i = 0; i < SIZE; ++i) {
            mTable[i].value = value;
        }
    }
    InternalNode(const InternalNode&) = delete; // disallow copy-construction
    InternalNode(InternalNode&&) = delete; // disallow move construction
    InternalNode& operator=(const InternalNode&) = delete; // disallow copy assignment
    InternalNode& operator=(InternalNode&&) = delete; // disallow move assignment
    ~InternalNode()
    {
        for (auto iter = mChildMask.beginOn(); iter; ++iter) {
            delete mTable[*iter].child;
        }
    }

    static uint32_t CoordToOffset(const Coord& ijk)
    {
        return (((ijk[0] & MASK) >> ChildT::TOTAL) << (2 * LOG2DIM)) +
               (((ijk[1] & MASK) >> ChildT::TOTAL) << (LOG2DIM)) +
                ((ijk[2] & MASK) >> ChildT::TOTAL);
    }

    static Coord OffsetToLocalCoord(uint32_t n)
    {
        NANOVDB_ASSERT(n < SIZE);
        const uint32_t m = n & ((1 << 2 * LOG2DIM) - 1);
        return Coord(n >> 2 * LOG2DIM, m >> LOG2DIM, m & ((1 << LOG2DIM) - 1));
    }

    void localToGlobalCoord(Coord& ijk) const
    {
        ijk <<= ChildT::TOTAL;
        ijk += mOrigin;
    }

    Coord offsetToGlobalCoord(uint32_t n) const
    {
        Coord ijk = InternalNode::OffsetToLocalCoord(n);
        this->localToGlobalCoord(ijk);
        return ijk;
    }

    template<typename AccT>
    bool isActiveAndCache(const Coord& ijk, AccT& acc) const
    {
        const uint32_t n = CoordToOffset(ijk);
        if (mChildMask.isOn(n)) {
            acc.insert(ijk, const_cast<ChildT*>(mTable[n].child));
            return mTable[n].child->isActiveAndCache(ijk, acc);
        }
        return mValueMask.isOn(n);
    }

    ValueType getFirstValue() const { return mChildMask.isOn(0) ? mTable[0].child->getFirstValue() : mTable[0].value; }
    ValueType getLastValue() const { return mChildMask.isOn(SIZE - 1) ? mTable[SIZE - 1].child->getLastValue() : mTable[SIZE - 1].value; }

    ValueType getValue(const Coord& ijk) const
    {
        const uint32_t n = CoordToOffset(ijk);
        if (mChildMask.isOn(n)) {
            return mTable[n].child->getValue(ijk);
        }
        return mTable[n].value;
    }

    template<typename AccT>
    ValueType getValueAndCache(const Coord& ijk, AccT& acc) const
    {
        const uint32_t n = CoordToOffset(ijk);
        if (mChildMask.isOn(n)) {
            acc.insert(ijk, const_cast<ChildT*>(mTable[n].child));
            return mTable[n].child->getValueAndCache(ijk, acc);
        }
        return mTable[n].value;
    }

    void setValue(const Coord& ijk, const ValueType& value)
    {
        const uint32_t n = CoordToOffset(ijk);
        ChildT*        child = nullptr;
        if (mChildMask.isOn(n)) {
            child = mTable[n].child;
        } else {
            child = new ChildT(ijk, mTable[n].value, mValueMask.isOn(n));
            mTable[n].child = child;
            mChildMask.setOn(n);
        }
        child->setValue(ijk, value);
    }

    template<typename AccT>
    void setValueAndCache(const Coord& ijk, const ValueType& value, AccT& acc)
    {
        const uint32_t n = CoordToOffset(ijk);
        ChildT*        child = nullptr;
        if (mChildMask.isOn(n)) {
            child = mTable[n].child;
        } else {
            child = new ChildT(ijk, mTable[n].value, mValueMask.isOn(n));
            mTable[n].child = child;
            mChildMask.setOn(n);
        }
        acc.insert(ijk, child);
        child->setValueAndCache(ijk, value, acc);
    }

    template<typename NodeT>
    uint32_t nodeCount() const
    {
        static_assert(is_same<ValueType, typename NodeT::ValueType>::value, "Node::getNodes: Invalid type");
        NANOVDB_ASSERT(NodeT::LEVEL < LEVEL);
        uint32_t sum = 0;
        if (is_same<NodeT, ChildT>::value) { //resolved at compile-time
            sum += mChildMask.countOn();
        } else {
            for (auto iter = mChildMask.beginOn(); iter; ++iter) {
                sum += mTable[*iter].child->template nodeCount<NodeT>();
            }
        }
        return sum;
    }

    template<typename NodeT>
    void getNodes(std::vector<NodeT*>& array)
    {
        static_assert(is_same<ValueType, typename NodeT::ValueType>::value, "Node::getNodes: Invalid type");
        NANOVDB_ASSERT(NodeT::LEVEL < LEVEL);
        for (auto iter = mChildMask.beginOn(); iter; ++iter) {
            if (is_same<NodeT, ChildT>::value) { //resolved at compile-time
                array.push_back(reinterpret_cast<NodeT*>(mTable[*iter].child));
            } else {
                mTable[*iter].child->getNodes(array);
            }
        }
    }

    void addChild(ChildT*& child)
    {
        NANOVDB_ASSERT(child && (child->mOrigin & ~MASK) == this->mOrigin);
        const uint32_t n = CoordToOffset(child->mOrigin);
        if (mChildMask.isOn(n)) {
            delete mTable[n].child;
        } else {
            mChildMask.setOn(n);
        }
        mTable[n].child = child;
        child = nullptr;
    }

    template<typename NodeT>
    void addNode(NodeT*& node)
    {
        if (is_same<NodeT, ChildT>::value) { //resolved at compile-time
            this->addChild(reinterpret_cast<ChildT*&>(node));
        } else {
            const uint32_t n = CoordToOffset(node->mOrigin);
            ChildT*        child = nullptr;
            if (mChildMask.isOn(n)) {
                child = mTable[n].child;
            } else {
                child = new ChildT(node->mOrigin, mTable[n].value, mValueMask.isOn(n));
                mTable[n].child = child;
                mChildMask.setOn(n);
            }
            child->addNode(node);
        }
    }

    template<typename T>
    typename std::enable_if<std::is_floating_point<T>::value>::type
    signedFloodFill(T outside);

    template<typename T>
    typename std::enable_if<!std::is_floating_point<T>::value>::type
    signedFloodFill(T) {} // no-op for none floating point values
}; // InternalNode

//================================================================================================

template<typename ChildT>
template<typename T>
inline typename std::enable_if<std::is_floating_point<T>::value>::type
InternalNode<ChildT>::signedFloodFill(T outside)
{
    const uint32_t first = *mChildMask.beginOn();
    if (first < NUM_VALUES) {
        bool xInside = mTable[first].child->getFirstValue() < 0;
        bool yInside = xInside, zInside = xInside;
        for (uint32_t x = 0; x != (1 << LOG2DIM); ++x) {
            const uint32_t x00 = x << (2 * LOG2DIM); // offset for block(x, 0, 0)
            if (mChildMask.isOn(x00)) {
                xInside = mTable[x00].child->getLastValue() < 0;
            }
            yInside = xInside;
            for (uint32_t y = 0; y != (1u << LOG2DIM); ++y) {
                const uint32_t xy0 = x00 + (y << LOG2DIM); // offset for block(x, y, 0)
                if (mChildMask.isOn(xy0))
                    yInside = mTable[xy0].child->getLastValue() < 0;
                zInside = yInside;
                for (uint32_t z = 0; z != (1 << LOG2DIM); ++z) {
                    const uint32_t xyz = xy0 + z; // offset for block(x, y, z)
                    if (mChildMask.isOn(xyz)) {
                        zInside = mTable[xyz].child->getLastValue() < 0;
                    } else {
                        mTable[xyz].value = zInside ? -outside : outside;
                    }
                }
            }
        }
    }
} // InternalNode::signedFloodFill

//================================================================================================

template<typename BuildT>
struct LeafNode
{
    using BuildType = BuildT;
    using ValueType = typename BuildToValueMap<BuildT>::type;
    static constexpr uint32_t LOG2DIM = 3;
    static constexpr uint32_t TOTAL = LOG2DIM; // needed by parent nodes
    static constexpr uint32_t DIM = 1u << TOTAL;
    static constexpr uint32_t SIZE = 1u << 3 * LOG2DIM; // total number of voxels represented by this node
    static constexpr int32_t  MASK = DIM - 1; // mask for bit operations
    static constexpr uint32_t LEVEL = 0; // level 0 = leaf
    static constexpr uint64_t NUM_VALUES = uint64_t(1) << (3 * TOTAL); // total voxel count represented by this node
    using NodeMaskType = Mask<LOG2DIM>;
    using NanoLeafT = typename NanoNode<BuildT, 0>::Type;

    Coord         mOrigin;
    Mask<LOG2DIM> mValueMask;
    ValueType     mValues[SIZE];
    union {
        NanoLeafT *mDstNode;
        uint64_t   mDstOffset;
    };

    LeafNode(const Coord& ijk, const ValueType& value, bool state)
        : mOrigin(ijk & ~MASK)
        , mValueMask(state) //invalid
        , mDstOffset(0)
    {
        ValueType*  target = mValues;
        uint32_t n = SIZE;
        while (n--) {
            *target++ = value;
        }
    }
    LeafNode(const LeafNode&) = delete; // disallow copy-construction
    LeafNode(LeafNode&&) = delete; // disallow move construction
    LeafNode& operator=(const LeafNode&) = delete; // disallow copy assignment
    LeafNode& operator=(LeafNode&&) = delete; // disallow move assignment
    ~LeafNode() = default;

    /// @brief Return the linear offset corresponding to the given coordinate
    static uint32_t CoordToOffset(const Coord& ijk)
    {
        return ((ijk[0] & MASK) << (2 * LOG2DIM)) + ((ijk[1] & MASK) << LOG2DIM) + (ijk[2] & MASK);
    }

    static Coord OffsetToLocalCoord(uint32_t n)
    {
        NANOVDB_ASSERT(n < SIZE);
        const int32_t m = n & ((1 << 2 * LOG2DIM) - 1);
        return Coord(n >> 2 * LOG2DIM, m >> LOG2DIM, m & MASK);
    }

    void localToGlobalCoord(Coord& ijk) const
    {
        ijk += mOrigin;
    }

    Coord offsetToGlobalCoord(uint32_t n) const
    {
        Coord ijk = LeafNode::OffsetToLocalCoord(n);
        this->localToGlobalCoord(ijk);
        return ijk;
    }

    template<typename AccT>
    bool isActiveAndCache(const Coord& ijk, const AccT&) const
    {
        return mValueMask.isOn(CoordToOffset(ijk));
    }

    ValueType getFirstValue() const { return mValues[0]; }
    ValueType getLastValue() const { return mValues[SIZE - 1]; }

    ValueType getValue(const Coord& ijk) const
    {
        return mValues[CoordToOffset(ijk)];
    }

    template<typename AccT>
    ValueType getValueAndCache(const Coord& ijk, const AccT&) const
    {
        return mValues[CoordToOffset(ijk)];
    }

    template<typename AccT>
    void setValueAndCache(const Coord& ijk, const ValueType& value, const AccT&)
    {
        const uint32_t n = CoordToOffset(ijk);
        mValueMask.setOn(n);
        mValues[n] = value;
    }

    void setValue(const Coord& ijk, const ValueType& value)
    {
        const uint32_t n = CoordToOffset(ijk);
        mValueMask.setOn(n);
        mValues[n] = value;
    }

    template<typename NodeT>
    void getNodes(std::vector<NodeT*>&) { NANOVDB_ASSERT(false); }

    template<typename NodeT>
    void addNode(NodeT*&) {}

    template<typename NodeT>
    uint32_t nodeCount() const
    {
        NANOVDB_ASSERT(false);// should never get called
        return 1;
    }

    template<typename T>
    typename std::enable_if<std::is_floating_point<T>::value>::type
    signedFloodFill(T outside);
    template<typename T>
    typename std::enable_if<!std::is_floating_point<T>::value>::type
    signedFloodFill(T) {} // no-op for none floating point values
}; // LeafNode<T>

//================================================================================================

template <>
struct LeafNode<ValueMask>
{
    using ValueType = bool;
    using BuildType = ValueMask;
    static constexpr uint32_t LOG2DIM = 3;
    static constexpr uint32_t TOTAL = LOG2DIM; // needed by parent nodes
    static constexpr uint32_t DIM = 1u << TOTAL;
    static constexpr uint32_t SIZE = 1u << 3 * LOG2DIM; // total number of voxels represented by this node
    static constexpr int32_t  MASK = DIM - 1; // mask for bit operations
    static constexpr uint32_t LEVEL = 0; // level 0 = leaf
    static constexpr uint64_t NUM_VALUES = uint64_t(1) << (3 * TOTAL); // total voxel count represented by this node
    using NodeMaskType = Mask<LOG2DIM>;
    using NanoLeafT = typename NanoNode<BuildType, 0>::Type;

    Coord         mOrigin;
    Mask<LOG2DIM> mValueMask;
    union {
        NanoLeafT *mDstNode;
        uint64_t   mDstOffset;
    };

    LeafNode(const Coord& ijk, const ValueType& value, bool state)
        : mOrigin(ijk & ~MASK)
        , mValueMask(state) //invalid
        , mDstOffset(0)
    {
    }
    LeafNode(const LeafNode&) = delete; // disallow copy-construction
    LeafNode(LeafNode&&) = delete; // disallow move construction
    LeafNode& operator=(const LeafNode&) = delete; // disallow copy assignment
    LeafNode& operator=(LeafNode&&) = delete; // disallow move assignment
    ~LeafNode() = default;

    /// @brief Return the linear offset corresponding to the given coordinate
    static uint32_t CoordToOffset(const Coord& ijk)
    {
        return ((ijk[0] & MASK) << (2 * LOG2DIM)) + ((ijk[1] & MASK) << LOG2DIM) + (ijk[2] & MASK);
    }

    static Coord OffsetToLocalCoord(uint32_t n)
    {
        NANOVDB_ASSERT(n < SIZE);
        const int32_t m = n & ((1 << 2 * LOG2DIM) - 1);
        return Coord(n >> 2 * LOG2DIM, m >> LOG2DIM, m & MASK);
    }

    void localToGlobalCoord(Coord& ijk) const
    {
        ijk += mOrigin;
    }

    Coord offsetToGlobalCoord(uint32_t n) const
    {
        Coord ijk = LeafNode::OffsetToLocalCoord(n);
        this->localToGlobalCoord(ijk);
        return ijk;
    }

    template<typename AccT>
    bool isActiveAndCache(const Coord& ijk, const AccT&) const
    {
        return mValueMask.isOn(CoordToOffset(ijk));
    }

    bool getFirstValue() const { return mValueMask.isOn(0); }
    bool getLastValue() const { return mValueMask.isOn(SIZE - 1); }

    bool getValue(const Coord& ijk) const
    {
        return mValueMask.isOn(CoordToOffset(ijk));
    }

    template<typename AccT>
    bool getValueAndCache(const Coord& ijk, const AccT&) const
    {
        return mValueMask.isOn(CoordToOffset(ijk));
    }

    template<typename AccT>
    void setValueAndCache(const Coord& ijk, bool value, const AccT&)
    {
        const uint32_t n = CoordToOffset(ijk);
        mValueMask.setOn(n);
    }

    void setValue(const Coord& ijk, bool value)
    {
        const uint32_t n = CoordToOffset(ijk);
        mValueMask.setOn(n);
    }

    template<typename NodeT>
    void getNodes(std::vector<NodeT*>&) { NANOVDB_ASSERT(false); }

    template<typename NodeT>
    void addNode(NodeT*&) {}

    template<typename NodeT>
    uint32_t nodeCount() const
    {
        NANOVDB_ASSERT(false);// should never get called
        return 1;
    }

    template<typename T>
    void signedFloodFill(T) {}// no-op

}; // LeafNode<ValueMask>

//================================================================================================

template <>
struct LeafNode<bool>
{
    using ValueType = bool;
    using BuildType = ValueMask;
    static constexpr uint32_t LOG2DIM = 3;
    static constexpr uint32_t TOTAL = LOG2DIM; // needed by parent nodes
    static constexpr uint32_t DIM = 1u << TOTAL;
    static constexpr uint32_t SIZE = 1u << 3 * LOG2DIM; // total number of voxels represented by this node
    static constexpr int32_t  MASK = DIM - 1; // mask for bit operations
    static constexpr uint32_t LEVEL = 0; // level 0 = leaf
    static constexpr uint64_t NUM_VALUES = uint64_t(1) << (3 * TOTAL); // total voxel count represented by this node
    using NodeMaskType = Mask<LOG2DIM>;
    using NanoLeafT = typename NanoNode<BuildType, 0>::Type;

    Coord         mOrigin;
    Mask<LOG2DIM> mValueMask, mValues;
    union {
        NanoLeafT *mDstNode;
        uint64_t   mDstOffset;
    };

    LeafNode(const Coord& ijk, bool value, bool state)
        : mOrigin(ijk & ~MASK)
        , mValueMask(state)
        , mValues(value)
        , mDstOffset(0)
    {
    }
    LeafNode(const LeafNode&) = delete; // disallow copy-construction
    LeafNode(LeafNode&&) = delete; // disallow move construction
    LeafNode& operator=(const LeafNode&) = delete; // disallow copy assignment
    LeafNode& operator=(LeafNode&&) = delete; // disallow move assignment
    ~LeafNode() = default;

    /// @brief Return the linear offset corresponding to the given coordinate
    static uint32_t CoordToOffset(const Coord& ijk)
    {
        return ((ijk[0] & MASK) << (2 * LOG2DIM)) + ((ijk[1] & MASK) << LOG2DIM) + (ijk[2] & MASK);
    }

    static Coord OffsetToLocalCoord(uint32_t n)
    {
        NANOVDB_ASSERT(n < SIZE);
        const int32_t m = n & ((1 << 2 * LOG2DIM) - 1);
        return Coord(n >> 2 * LOG2DIM, m >> LOG2DIM, m & MASK);
    }

    void localToGlobalCoord(Coord& ijk) const
    {
        ijk += mOrigin;
    }

    Coord offsetToGlobalCoord(uint32_t n) const
    {
        Coord ijk = LeafNode::OffsetToLocalCoord(n);
        this->localToGlobalCoord(ijk);
        return ijk;
    }

    template<typename AccT>
    bool isActiveAndCache(const Coord& ijk, const AccT&) const
    {
        return mValueMask.isOn(CoordToOffset(ijk));
    }

    bool getFirstValue() const { return mValues.isOn(0); }
    bool getLastValue() const { return mValues.isOn(SIZE - 1); }

    bool getValue(const Coord& ijk) const
    {
        return mValues.isOn(CoordToOffset(ijk));
    }

    template<typename AccT>
    bool getValueAndCache(const Coord& ijk, const AccT&) const
    {
        return mValues.isOn(CoordToOffset(ijk));
    }

    template<typename AccT>
    void setValueAndCache(const Coord& ijk, bool value, const AccT&)
    {
        const uint32_t n = CoordToOffset(ijk);
        mValueMask.setOn(n);
        mValues.setOn(n);
    }

    void setValue(const Coord& ijk, bool value)
    {
        const uint32_t n = CoordToOffset(ijk);
        mValueMask.setOn(n);
        mValues.setOn(n);
    }

    template<typename NodeT>
    void getNodes(std::vector<NodeT*>&) { NANOVDB_ASSERT(false); }

    template<typename NodeT>
    void addNode(NodeT*&) {}

    template<typename NodeT>
    uint32_t nodeCount() const
    {
        NANOVDB_ASSERT(false);// should never get called
        return 1;
    }

    template<typename T>
    void signedFloodFill(T) {}// no-op

}; // LeafNode<bool>

//================================================================================================

template<typename BuildT>
template<typename T>
inline typename std::enable_if<std::is_floating_point<T>::value>::type
LeafNode<BuildT>::signedFloodFill(T outside)
{
    const uint32_t first = *mValueMask.beginOn();
    if (first < SIZE) {
        bool xInside = mValues[first] < 0, yInside = xInside, zInside = xInside;
        for (uint32_t x = 0; x != DIM; ++x) {
            const uint32_t x00 = x << (2 * LOG2DIM);
            if (mValueMask.isOn(x00))
                xInside = mValues[x00] < 0; // element(x, 0, 0)
            yInside = xInside;
            for (uint32_t y = 0; y != DIM; ++y) {
                const uint32_t xy0 = x00 + (y << LOG2DIM);
                if (mValueMask.isOn(xy0))
                    yInside = mValues[xy0] < 0; // element(x, y, 0)
                zInside = yInside;
                for (uint32_t z = 0; z != (1 << LOG2DIM); ++z) {
                    const uint32_t xyz = xy0 + z; // element(x, y, z)
                    if (mValueMask.isOn(xyz)) {
                        zInside = mValues[xyz] < 0;
                    } else {
                        mValues[xyz] = zInside ? -outside : outside;
                    }
                }
            }
        }
    }
} // LeafNode<T>::signedFloodFill

//================================================================================================

template<typename BuildT>
struct ValueAccessor
{
    using ValueT = typename BuildToValueMap<BuildT>::type;
    using LeafT = build::LeafNode<BuildT>;
    using Node1 = build::InternalNode<LeafT>;
    using Node2 = build::InternalNode<Node1>;
    using RootT = build::RootNode<Node2>;

    ValueAccessor(RootT& root)
        : mKeys{Coord(Maximum<int>::value()), Coord(Maximum<int>::value()), Coord(Maximum<int>::value())}
        , mNode{nullptr, nullptr, nullptr, &root}
    {
    }
    template<typename NodeT>
    bool isCached(const Coord& ijk) const
    {
        return (ijk[0] & ~NodeT::MASK) == mKeys[NodeT::LEVEL][0] &&
               (ijk[1] & ~NodeT::MASK) == mKeys[NodeT::LEVEL][1] &&
               (ijk[2] & ~NodeT::MASK) == mKeys[NodeT::LEVEL][2];
    }
    ValueT getValue(const Coord& ijk)
    {
        if (this->isCached<LeafT>(ijk)) {
            return ((LeafT*)mNode[0])->getValueAndCache(ijk, *this);
        } else if (this->isCached<Node1>(ijk)) {
            return ((Node1*)mNode[1])->getValueAndCache(ijk, *this);
        } else if (this->isCached<Node2>(ijk)) {
            return ((Node2*)mNode[2])->getValueAndCache(ijk, *this);
        }
        return ((RootT*)mNode[3])->getValueAndCache(ijk, *this);
    }
    /// @brief Sets value in a leaf node and returns it.
    LeafT* setValue(const Coord& ijk, const ValueT& value)
    {
        if (this->isCached<LeafT>(ijk)) {
            ((LeafT*)mNode[0])->setValueAndCache(ijk, value, *this);
        } else if (this->isCached<Node1>(ijk)) {
            ((Node1*)mNode[1])->setValueAndCache(ijk, value, *this);
        } else if (this->isCached<Node2>(ijk)) {
            ((Node2*)mNode[2])->setValueAndCache(ijk, value, *this);
        } else {
            ((RootT*)mNode[3])->setValueAndCache(ijk, value, *this);
        }
        NANOVDB_ASSERT(this->isCached<LeafT>(ijk));
        return (LeafT*)mNode[0];
    }
    bool isActive(const Coord& ijk)
    {
        if (this->isCached<LeafT>(ijk)) {
            return ((LeafT*)mNode[0])->isActiveAndCache(ijk, *this);
        } else if (this->isCached<Node1>(ijk)) {
            return ((Node1*)mNode[1])->isActiveAndCache(ijk, *this);
        } else if (this->isCached<Node2>(ijk)) {
            return ((Node2*)mNode[2])->isActiveAndCache(ijk, *this);
        }
        return ((RootT*)mNode[3])->isActiveAndCache(ijk, *this);
    }
    bool isValueOn(const Coord& ijk) { return this->isActive(ijk); }
    template<typename NodeT>
    void insert(const Coord& ijk, NodeT* node)
    {
        mKeys[NodeT::LEVEL] = ijk & ~NodeT::MASK;
        mNode[NodeT::LEVEL] = node;
    }
    Coord mKeys[3];
    void* mNode[4];
}; // ValueAccessor

} // namespace build

} // namespace nanovdb

#endif // NANOVDB_BASEBUILDER_H_HAS_BEEN_INCLUDED
