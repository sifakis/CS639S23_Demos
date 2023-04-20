// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

/*!
    \file GridBuilder.h

    \author Ken Museth

    \date June 26, 2020

    \brief Generates a NanoVDB grid from any volume or function.

    \note This is only intended as a simple tool to generate nanovdb grids without
          any dependency on openvdb.
*/

#ifndef NANOVDB_GRIDBUILDER_H_HAS_BEEN_INCLUDED
#define NANOVDB_GRIDBUILDER_H_HAS_BEEN_INCLUDED

#include "BaseBuilder.h"
#include "GridHandle.h"
#include "GridStats.h"
#include "GridChecksum.h"
#include "Range.h"
#include "Invoke.h"
#include "ForEach.h"
#include "Reduce.h"
#include "DitherLUT.h"// for nanovdb::DitherLUT

#include <limits>
#include <sstream> // for stringstream
#include <vector>
#include <cstring> // for memcpy

namespace nanovdb {

/// @brief Compression oracle based on absolute difference
class AbsDiff
{
    float mTolerance;// absolute error tolerance
public:
    /// @note The default value of -1 means it's un-initialized!
    AbsDiff(float tolerance = -1.0f) : mTolerance(tolerance) {}
    AbsDiff(const AbsDiff&) = default;
    void  setTolerance(float tolerance) { mTolerance = tolerance; }
    float getTolerance() const { return mTolerance; }
    /// @brief Return true if the approximate value is within the accepted
    ///        absolute error bounds of the exact value.
    ///
    /// @details Required member method
    bool  operator()(float exact, float approx) const
    {
        return Abs(exact - approx) <= mTolerance;
    }
};// AbsDiff

inline std::ostream& operator<<(std::ostream& os, const AbsDiff& diff)
{
    os << "Absolute tolerance: " << diff.getTolerance();
    return os;
}

/// @brief Compression oracle based on relative difference
class RelDiff
{
    float mTolerance;// relative error tolerance
public:
    /// @note The default value of -1 means it's un-initialized!
    RelDiff(float tolerance = -1.0f) : mTolerance(tolerance) {}
    RelDiff(const RelDiff&) = default;
    void  setTolerance(float tolerance) { mTolerance = tolerance; }
    float getTolerance() const { return mTolerance; }
    /// @brief Return true if the approximate value is within the accepted
    ///        relative error bounds of the exact value.
    ///
    /// @details Required member method
    bool  operator()(float exact, float approx) const
    {
        return  Abs(exact - approx)/Max(Abs(exact), Abs(approx)) <= mTolerance;
    }
};// RelDiff

inline std::ostream& operator<<(std::ostream& os, const RelDiff& diff)
{
    os << "Relative tolerance: " << diff.getTolerance();
    return os;
}

/// @brief Allows for the construction of NanoVDB grids without any dependency
template<typename BuildT, typename StatsT = Stats<typename BuildToValueMap<BuildT>::type>>
class GridBuilder
{
    struct Codec {float min, max; uint16_t log2, size;};// used for adaptive bit-rate quantization

    using ValueT = typename BuildToValueMap<BuildT>::type;
    using SrcNode0 = build::LeafNode<BuildT>;
    using SrcNode1 = build::InternalNode<SrcNode0>;
    using SrcNode2 = build::InternalNode<SrcNode1>;
    using SrcRootT = build::RootNode<SrcNode2>;

    using DstNode0 = NanoLeaf< BuildT>;// nanovdb::LeafNode<ValueT>; // leaf
    using DstNode1 = NanoLower<BuildT>;// nanovdb::InternalNode<DstNode0>; // lower
    using DstNode2 = NanoUpper<BuildT>;// nanovdb::InternalNode<DstNode1>; // upper
    using DstRootT = NanoRoot< BuildT>;// nanovdb::RootNode<DstNode2>;
    using DstTreeT = NanoTree< BuildT>;
    using DstGridT = NanoGrid< BuildT>;

    ValueT                   mDelta; // skip node if: node.max < -mDelta || node.min > mDelta
    uint8_t*                 mBufferPtr;// pointer to the beginning of the buffer
    uint64_t                 mBufferOffsets[9];//grid, tree, root, upper, lower, leafs, meta data, blind data, buffer size
    int                      mVerbose;
    uint64_t                 mBlindDataSize;
    SrcRootT                 mRoot;// this root supports random write
    std::vector<SrcNode0*>   mArray0; // leaf nodes
    std::vector<SrcNode1*>   mArray1; // lower internal nodes
    std::vector<SrcNode2*>   mArray2; // upper internal nodes
    std::unique_ptr<Codec[]> mCodec;// defines a codec per leaf node
    GridClass                mGridClass;
    StatsMode                mStats;
    ChecksumMode             mChecksum;
    bool                     mDitherOn;

    // Below are private methods use to serialize nodes into NanoVDB
    template< typename OracleT, typename BufferT>
    GridHandle<BufferT> initHandle(const OracleT &oracle, const BufferT& buffer);

    template <typename T, typename OracleT>
    inline typename std::enable_if<!is_same<T, FpN>::value>::type
    compression(uint64_t&, OracleT) {}// no-op

    template <typename T, typename OracleT>
    inline typename std::enable_if<is_same<T, FpN>::value>::type
    compression(uint64_t &offset, OracleT oracle);

    template<typename T>
    typename std::enable_if<!is_same<bool,      typename T::BuildType>::value &&
                            !is_same<ValueMask, typename T::BuildType>::value &&
                            !is_same<Fp4,       typename T::BuildType>::value &&
                            !is_same<Fp8,       typename T::BuildType>::value &&
                            !is_same<Fp16,      typename T::BuildType>::value &&
                            !is_same<FpN,       typename T::BuildType>::value>::type
    processLeafs(std::vector<T*>&);

    template<typename T>
    typename std::enable_if<is_same<Fp4,  typename T::BuildType>::value ||
                            is_same<Fp8,  typename T::BuildType>::value ||
                            is_same<Fp16, typename T::BuildType>::value>::type
    processLeafs(std::vector<T*>&);

    template<typename T>
    typename std::enable_if<is_same<bool, typename T::BuildType>::value>::type
    processLeafs(std::vector<T*>&);

    template<typename T>
    typename std::enable_if<is_same<ValueMask, typename T::BuildType>::value>::type
    processLeafs(std::vector<T*>&);

    template<typename T>
    typename std::enable_if<is_same<FpN, typename T::BuildType>::value>::type
    processLeafs(std::vector<T*>&);

    template<typename SrcNodeT>
    void processNodes(std::vector<SrcNodeT*>&);

    DstRootT* processRoot();

    DstTreeT* processTree();

    DstGridT* processGrid(const Map&, const std::string&);

    template<typename T, typename FlagT>
    typename std::enable_if<!std::is_floating_point<T>::value>::type
    setFlag(const T&, const T&, FlagT& flag) const { flag &= ~FlagT(1); } // unset first bit

    template<typename T, typename FlagT>
    typename std::enable_if<std::is_floating_point<T>::value>::type
    setFlag(const T& min, const T& max, FlagT& flag) const;

public:

    using ValueAccessorT = build::ValueAccessor<BuildT>;

    GridBuilder(ValueT background = ValueT(),
                GridClass gClass = GridClass::Unknown,
                uint64_t blindDataSize = 0);

    ValueAccessorT getAccessor() { return ValueAccessorT(mRoot); }

    /// @brief Performs multi-threaded bottom-up signed-distance flood-filling and changes GridClass to LevelSet
    ///
    /// @warning Only call this method once this GridBuilder contains a valid signed distance field
    void sdfToLevelSet();

    /// @brief Performs multi-threaded bottom-up signed-distance flood-filling followed by level-set -> FOG volume
    ///        conversion. It also changes the GridClass to FogVolume
    ///
    /// @warning Only call this method once this GridBuilder contains a valid signed distance field
    void sdfToFog();

    void setVerbose(int mode = 1) { mVerbose = mode; }

    void enableDithering(bool on = true) { mDitherOn = on; }

    void setStats(StatsMode mode = StatsMode::Default) { mStats = mode; }

    void setChecksum(ChecksumMode mode = ChecksumMode::Default) { mChecksum = mode; }

    void setGridClass(GridClass mode = GridClass::Unknown) { mGridClass = mode; }

    /// @brief Return an instance of a GridHandle (invoking move semantics)
    template<typename OracleT = AbsDiff, typename BufferT = HostBuffer>
    GridHandle<BufferT> getHandle(double             voxelSize = 1.0,
                                  const Vec3d&       gridOrigin = Vec3d(0),
                                  const std::string& name = "",
                                  const OracleT&     oracle = OracleT(),
                                  const BufferT&     buffer = BufferT());

    /// @brief Return an instance of a GridHandle (invoking move semantics)
    template<typename OracleT = AbsDiff, typename BufferT = HostBuffer>
    GridHandle<BufferT> getHandle(const Map&         map,
                                  const std::string& name = "",
                                  const OracleT&     oracle = OracleT(),
                                  const BufferT&     buffer = BufferT());

    /// @brief Sets grids values in domain of the @a bbox to those returned by the specified @a func with the
    ///        expected signature [](const Coord&)->ValueT.
    ///
    /// @note If @a func returns a value equal to the background value (specified in the constructor) at a
    ///       specific voxel coordinate, then the active state of that coordinate is left off! Else the value
    ///       value is set and the active state is on. This is done to allow for sparse grids to be generated.
    ///
    /// @param func  Functor used to evaluate the grid values in the @a bbox
    /// @param bbox  Coordinate bounding-box over which the grid values will be set.
    /// @param delta Specifies a lower threshold value for rendering (optional). Typically equals the voxel size
    ///              for level sets and otherwise it's zero.
    template<typename Func>
    void operator()(const Func& func, const CoordBBox& bbox, ValueT delta = ValueT(0));

}; // GridBuilder

//================================================================================================

template<typename BuildT, typename StatsT>
GridBuilder<BuildT, StatsT>::
GridBuilder(ValueT background, GridClass gClass, uint64_t blindDataSize)
    : mDelta(0)
    , mVerbose(0)
    , mBlindDataSize(blindDataSize)
    , mRoot(background)
    , mGridClass(gClass)
    , mStats(StatsMode::Default)
    , mChecksum(ChecksumMode::Default)
    , mDitherOn(false)
{
}

template<typename BuildT, typename StatsT>
template<typename Func>
void GridBuilder<BuildT, StatsT>::
operator()(const Func& func, const CoordBBox& voxelBBox, ValueT delta)
{
    static_assert(is_same<ValueT, typename std::result_of<Func(const Coord&)>::type>::value, "GridBuilder: mismatched ValueType");
    mDelta = delta; // delta = voxel size for level sets, else 0
    const CoordBBox leafBBox(voxelBBox[0] >> SrcNode0::TOTAL, voxelBBox[1] >> SrcNode0::TOTAL);
    std::mutex mutex;
    forEach(leafBBox, [&](const CoordBBox& b) {
        SrcNode0* leaf = nullptr;
        for (auto it = b.begin(); it; ++it) {
            Coord min(*it << SrcNode0::TOTAL), max(min + Coord(SrcNode0::DIM - 1));
            const CoordBBox bbox(min.maxComponent(voxelBBox.min()),
                                 max.minComponent(voxelBBox.max()));// crop
            if (leaf == nullptr) {
                leaf = new SrcNode0(bbox[0], mRoot.mBackground, false);
            } else {
                leaf->mOrigin = bbox[0] & ~SrcNode0::MASK;
                NANOVDB_ASSERT(leaf->mValueMask.isOff());
            }
            leaf->mDstOffset = 0;// no prune
            for (auto ijk = bbox.begin(); ijk; ++ijk) {
                const auto v = func(*ijk);
                if (v == mRoot.mBackground) {// don't insert background values
                    continue;
                }
                leaf->setValue(*ijk, v);
            }
            if (!leaf->mValueMask.isOff()) {// has active values
                if (leaf->mValueMask.isOn()) {// only active values
                    const auto first = leaf->getFirstValue();
                    int n=1;
                    while (n<512) {// 8^3 = 512
                        if (leaf->mValues[n++] != first) break;
                    }
                    if (n == 512) leaf->mDstOffset = 1;// prune below
                }
                std::lock_guard<std::mutex> guard(mutex);
                NANOVDB_ASSERT(leaf != nullptr);
                mRoot.addNode(leaf);
                NANOVDB_ASSERT(leaf == nullptr);
            }
        }// loop over sub-part of leafBBox
        if (leaf) {
            delete leaf;
        }
    }); // kernel

    // Prune leaf and tile nodes
    for (auto it2 = mRoot.mTable.begin(); it2 != mRoot.mTable.end(); ++it2) {
        if (auto *upper = it2->second.child) {//upper level internal node
            for (auto it1 = upper->mChildMask.beginOn(); it1; ++it1) {
                auto *lower = upper->mTable[*it1].child;// lower level internal node
                for (auto it0 = lower->mChildMask.beginOn(); it0; ++it0) {
                    auto *leaf = lower->mTable[*it0].child;// leaf nodes
                    if (leaf->mDstOffset) {
                        lower->mTable[*it0].value = leaf->getFirstValue();
                        lower->mChildMask.setOff(*it0);
                        lower->mValueMask.setOn(*it0);
                        delete leaf;
                    }
                }// loop over leaf nodes
                if (lower->mChildMask.isOff()) {//only tiles
                    const auto first = lower->getFirstValue();
                    int n=1;
                    while (n < 4096) {// 16^3 = 4096
                        if (lower->mTable[n++].value != first) break;
                    }
                    if (n == 4096) {// identical tile values so prune
                        upper->mTable[*it1].value = first;
                        upper->mChildMask.setOff(*it1);
                        upper->mValueMask.setOn(*it1);
                        delete lower;
                    }
                }
            }// loop over lower internal nodes
            if (upper->mChildMask.isOff()) {//only tiles
                const auto first = upper->getFirstValue();
                int n=1;
                while (n < 32768) {// 32^3 = 32768
                    if (upper->mTable[n++].value != first) break;
                }
                if (n == 32768) {// identical tile values so prune
                    it2->second.value = first;
                    it2->second.state = upper->mValueMask.isOn();
                    it2->second.child = nullptr;
                    delete upper;
                }
            }
        }// is child node of the root
    }// loop over root table
}

//================================================================================================

template<typename BuildT, typename StatsT>
template<typename OracleT, typename BufferT>
GridHandle<BufferT> GridBuilder<BuildT, StatsT>::
initHandle(const OracleT &oracle, const BufferT& buffer)
{
    mArray0.clear();
    mArray1.clear();
    mArray2.clear();
    mArray0.reserve(mRoot.template nodeCount<SrcNode0>());
    mArray1.reserve(mRoot.template nodeCount<SrcNode1>());
    mArray2.reserve(mRoot.template nodeCount<SrcNode2>());

    uint64_t offset[3] = {0};
    for (auto it2 = mRoot.mTable.begin(); it2 != mRoot.mTable.end(); ++it2) {
        if (SrcNode2 *upper = it2->second.child) {
            upper->mDstOffset = offset[2];
            mArray2.emplace_back(upper);
            offset[2] += DstNode2::memUsage();
            for (auto it1 = upper->mChildMask.beginOn(); it1; ++it1) {
                SrcNode1 *lower = upper->mTable[*it1].child;
                lower->mDstOffset = offset[1];
                mArray1.emplace_back(lower);
                offset[1] += DstNode1::memUsage();
                for (auto it0 = lower->mChildMask.beginOn(); it0; ++it0) {
                    SrcNode0 *leaf = lower->mTable[*it0].child;
                    leaf->mDstOffset = offset[0];// dummy if BuildT = FpN
                    mArray0.emplace_back(leaf);
                    offset[0] += sizeof(DstNode0);// dummy if BuildT = FpN
                }// loop over leaf nodes
            }// loop over lower internal nodes
        }// is child node of the root
    }// loop over root table

    this->template compression<BuildT, OracleT>(offset[0], oracle);// no-op unless BuildT = FpN

    mBufferOffsets[0] = 0;// grid is always stored at the start of the buffer!
    mBufferOffsets[1] = DstGridT::memUsage(); // tree
    mBufferOffsets[2] = DstTreeT::memUsage(); // root
    mBufferOffsets[3] = DstRootT::memUsage(static_cast<uint32_t>(mRoot.mTable.size())); // upper internal nodes
    mBufferOffsets[4] = offset[2]; // lower internal nodes
    mBufferOffsets[5] = offset[1]; // leaf nodes
    mBufferOffsets[6] = offset[0]; // blind meta data
    mBufferOffsets[7] = GridBlindMetaData::memUsage(mBlindDataSize > 0 ? 1 : 0); // blind data
    mBufferOffsets[8] = mBlindDataSize;// end of buffer

    // Compute the prefixed sum
    for (int i = 2; i < 9; ++i) {
        mBufferOffsets[i] += mBufferOffsets[i - 1];
    }

    GridHandle<BufferT> handle(BufferT::create(mBufferOffsets[8], &buffer));
    mBufferPtr = handle.data();
    return handle;
} // GridBuilder::initHandle

//================================================================================================

template<typename BuildT, typename StatsT>
template <typename T, typename OracleT>
inline typename std::enable_if<is_same<T, FpN>::value>::type
GridBuilder<BuildT, StatsT>::compression(uint64_t &offset, OracleT oracle)
{
    static_assert(is_same<FpN  , BuildT>::value, "compression: expected BuildT == float");
    static_assert(is_same<float, ValueT>::value, "compression: expected ValueT == float");
    if (is_same<AbsDiff, OracleT>::value && oracle.getTolerance() < 0.0f) {// default tolerance for level set and fog volumes
        if (mGridClass == GridClass::LevelSet) {
            static const float halfWidth = 3.0f;
            oracle.setTolerance(0.1f * mRoot.mBackground / halfWidth);// range of ls: [-3dx; 3dx]
        } else if (mGridClass == GridClass::FogVolume) {
            oracle.setTolerance(0.01f);// range of FOG volumes: [0;1]
        } else {
            oracle.setTolerance(0.0f);
        }
    }

    const size_t size = mArray0.size();
    mCodec.reset(new Codec[size]);

    DitherLUT lut(mDitherOn);
    auto kernel = [&](const Range1D &r) {
        for (auto i=r.begin(); i!=r.end(); ++i) {
            const float *data = mArray0[i]->mValues;
            float min = std::numeric_limits<float>::max(), max = -min;
            for (int j=0; j<512; ++j) {
                float v = data[j];
                if (v<min) min = v;
                if (v>max) max = v;
            }
            mCodec[i].min = min;
            mCodec[i].max = max;
            const float range = max - min;
            uint16_t logBitWidth = 0;// 0,1,2,3,4 => 1,2,4,8,16 bits
            while (range > 0.0f && logBitWidth < 4u) {
                const uint32_t mask = (uint32_t(1) << (uint32_t(1) << logBitWidth)) - 1u;
                const float encode  = mask/range;
                const float decode  = range/mask;
                int j = 0;
                do {
                    const float exact  = data[j];// exact value
                    const uint32_t code = uint32_t(encode*(exact - min) + lut(j));
                    const float approx = code * decode + min;// approximate value
                    j += oracle(exact, approx) ? 1 : 513;
                } while(j < 512);
                if (j == 512) break;
                ++logBitWidth;
            }
            mCodec[i].log2 = logBitWidth;
            mCodec[i].size = DstNode0::DataType::memUsage(1u << logBitWidth);
        }
    };// kernel
    forEach(0, size, 4, kernel);

    if (mVerbose) {
        uint32_t counters[5+1] = {0};
        ++counters[mCodec[0].log2];
        for (size_t i=1; i<size; ++i) {
            ++counters[mCodec[i].log2];
            mArray0[i]->mDstOffset = mArray0[i-1]->mDstOffset + mCodec[i-1].size;
        }
        std::cout << "\n" << oracle << std::endl;
        std::cout << "Dithering: " << (mDitherOn ? "enabled" : "disabled") << std::endl;
        float avg = 0.0f;
        for (uint32_t i=0; i<=5; ++i) {
            if (uint32_t n = counters[i]) {
                avg += n * float(1 << i);
                printf("%2i bits: %6u leaf nodes, i.e. %4.1f%%\n",1<<i, n, 100.0f*n/float(size));
            }
        }
        printf("%4.1f bits per value on average\n", avg/float(size));
    } else {
        for (size_t i=1; i<size; ++i) {
            mArray0[i]->mDstOffset = mArray0[i-1]->mDstOffset + mCodec[i-1].size;
        }
    }
    offset = mArray0[size-1]->mDstOffset + mCodec[size-1].size;
}// GridBuilder::compression

//================================================================================================

template<typename BuildT, typename StatsT>
void GridBuilder<BuildT, StatsT>::
    sdfToLevelSet()
{
    mArray0.clear();
    mArray1.clear();
    mArray2.clear();
    mArray0.reserve(mRoot.template nodeCount<SrcNode0>());
    mArray1.reserve(mRoot.template nodeCount<SrcNode1>());
    mArray2.reserve(mRoot.template nodeCount<SrcNode2>());

    for (auto it2 = mRoot.mTable.begin(); it2 != mRoot.mTable.end(); ++it2) {
        if (SrcNode2 *upper = it2->second.child) {
            mArray2.emplace_back(upper);
            for (auto it1 = upper->mChildMask.beginOn(); it1; ++it1) {
                SrcNode1 *lower = upper->mTable[*it1].child;
                mArray1.emplace_back(lower);
                for (auto it0 = lower->mChildMask.beginOn(); it0; ++it0) {
                    mArray0.emplace_back(lower->mTable[*it0].child);
                }// loop over leaf nodes
            }// loop over lower internal nodes
        }// is child node of the root
    }// loop over root table

    // Note that the bottom-up flood filling is essential
    const ValueT outside = mRoot.mBackground;
    forEach(mArray0, 8, [&](const Range1D& r) {
        for (auto i = r.begin(); i != r.end(); ++i)
            mArray0[i]->signedFloodFill(outside);
    });
    forEach(mArray1, 1, [&](const Range1D& r) {
        for (auto i = r.begin(); i != r.end(); ++i)
            mArray1[i]->signedFloodFill(outside);
    });
    forEach(mArray2, 1, [&](const Range1D& r) {
        for (auto i = r.begin(); i != r.end(); ++i)
            mArray2[i]->signedFloodFill(outside);
    });
    mRoot.signedFloodFill(outside);
    mGridClass = GridClass::LevelSet;
} // GridBuilder::sdfToLevelSet

//================================================================================================

template<typename BuildT, typename StatsT>
template<typename OracleT, typename BufferT>
GridHandle<BufferT> GridBuilder<BuildT, StatsT>::
    getHandle(double             dx, //voxel size
              const Vec3d&       p0, // origin
              const std::string& name,
              const OracleT&     oracle,
              const BufferT&     buffer)
{
    if (dx <= 0) {
        throw std::runtime_error("GridBuilder: voxel size is zero or negative");
    }
    Map map; // affine map
    map.set(dx, p0, 1.0);
    return this->getHandle(map, name, oracle, buffer);
} // GridBuilder::getHandle

//================================================================================================

template<typename BuildT, typename StatsT>
template< typename OracleT, typename BufferT>
GridHandle<BufferT> GridBuilder<BuildT, StatsT>::
    getHandle(const Map&         map,
              const std::string& name,
              const OracleT&     oracle,
              const BufferT&     buffer)
{
    if (mGridClass == GridClass::LevelSet && !is_floating_point<ValueT>::value) {
        throw std::runtime_error("Level sets are expected to be floating point types");
    } else if (mGridClass == GridClass::FogVolume && !is_floating_point<ValueT>::value) {
        throw std::runtime_error("Fog volumes are expected to be floating point types");
    }

    auto handle = this->template initHandle<OracleT, BufferT>(oracle, buffer);// initialize the arrays of nodes

    this->processLeafs(mArray0);

    this->processNodes(mArray1);

    this->processNodes(mArray2);

    auto *grid = this->processGrid(map, name);

    gridStats(*grid, mStats);

    updateChecksum(*grid, mChecksum);

    return handle;
} // GridBuilder::getHandle

//================================================================================================

template<typename BuildT, typename StatsT>
template<typename T, typename FlagT>
inline typename std::enable_if<std::is_floating_point<T>::value>::type
GridBuilder<BuildT, StatsT>::
    setFlag(const T& min, const T& max, FlagT& flag) const
{
    if (mDelta > 0 && (min > mDelta || max < -mDelta)) {
        flag |= FlagT(1); // set first bit
    } else {
        flag &= ~FlagT(1); // unset first bit
    }
}

//================================================================================================

template<typename BuildT, typename StatsT>
inline void GridBuilder<BuildT, StatsT>::
    sdfToFog()
{
    this->sdfToLevelSet(); // performs signed flood fill

    const ValueT d = -mRoot.mBackground, w = 1.0f / d;
    auto        op = [&](ValueT& v) -> bool {
        if (v > ValueT(0)) {
            v = ValueT(0);
            return false;
        }
        v = v > d ? v * w : ValueT(1);
        return true;
    };
    auto kernel0 = [&](const Range1D& r) {
        for (auto i = r.begin(); i != r.end(); ++i) {
            SrcNode0* node = mArray0[i];
            for (uint32_t i = 0; i < SrcNode0::SIZE; ++i)
                node->mValueMask.set(i, op(node->mValues[i]));
        }
    };
    auto kernel1 = [&](const Range1D& r) {
        for (auto i = r.begin(); i != r.end(); ++i) {
            SrcNode1* node = mArray1[i];
            for (uint32_t i = 0; i < SrcNode1::SIZE; ++i) {
                if (node->mChildMask.isOn(i)) {
                    SrcNode0* leaf = node->mTable[i].child;
                    if (leaf->mValueMask.isOff()) {
                        node->mTable[i].value = leaf->getFirstValue();
                        node->mChildMask.setOff(i);
                        delete leaf;
                    }
                } else {
                    node->mValueMask.set(i, op(node->mTable[i].value));
                }
            }
        }
    };
    auto kernel2 = [&](const Range1D& r) {
        for (auto i = r.begin(); i != r.end(); ++i) {
            SrcNode2* node = mArray2[i];
            for (uint32_t i = 0; i < SrcNode2::SIZE; ++i) {
                if (node->mChildMask.isOn(i)) {
                    SrcNode1* child = node->mTable[i].child;
                    if (child->mChildMask.isOff() && child->mValueMask.isOff()) {
                        node->mTable[i].value = child->getFirstValue();
                        node->mChildMask.setOff(i);
                        delete child;
                    }
                } else {
                    node->mValueMask.set(i, op(node->mTable[i].value));
                }
            }
        }
    };
    forEach(mArray0, 8, kernel0);
    forEach(mArray1, 1, kernel1);
    forEach(mArray2, 1, kernel2);

    for (auto it = mRoot.mTable.begin(); it != mRoot.mTable.end(); ++it) {
        SrcNode2* child = it->second.child;
        if (child == nullptr) {
            it->second.state = op(it->second.value);
        } else if (child->mChildMask.isOff() && child->mValueMask.isOff()) {
            it->second.value = child->getFirstValue();
            it->second.state = false;
            it->second.child = nullptr;
            delete child;
        }
    }
    mGridClass = GridClass::FogVolume;
} // GridBuilder::sdfToFog

//================================================================================================

template<typename BuildT, typename StatsT>
template<typename T>
inline typename std::enable_if<!is_same<bool,      typename T::BuildType>::value &&
                               !is_same<ValueMask, typename T::BuildType>::value &&
                               !is_same<Fp4,       typename T::BuildType>::value &&
                               !is_same<Fp8,       typename T::BuildType>::value &&
                               !is_same<Fp16,      typename T::BuildType>::value &&
                               !is_same<FpN,       typename T::BuildType>::value>::type
GridBuilder<BuildT, StatsT>::
    processLeafs(std::vector<T*>& srcLeafs)
{
    forEach(srcLeafs, 8, [&](const Range1D& r) {
        auto *ptr = mBufferPtr + mBufferOffsets[5];
        for (auto i = r.begin(); i != r.end(); ++i) {
            auto *srcLeaf = srcLeafs[i];
            auto *dstLeaf = PtrAdd<DstNode0>(ptr, srcLeaf->mDstOffset);
            auto *data = dstLeaf->data();
            if (DstNode0::DataType::padding()>0u) {
                std::memset(data, 0, DstNode0::DataType::memUsage());
            } else {
                data->mBBoxDif[0] = 0u;
                data->mBBoxDif[1] = 0u;
                data->mBBoxDif[2] = 0u;
                data->mFlags = 0u;// enable rendering, no bbox, no stats
                data->mMinimum = data->mMaximum = ValueT();
                data->mAverage = data->mStdDevi = 0;
            }
            srcLeaf->mDstNode = dstLeaf;
            data->mBBoxMin = srcLeaf->mOrigin; // copy origin of node
            data->mValueMask = srcLeaf->mValueMask; // copy value mask
            const ValueT* src = srcLeaf->mValues;
            for (ValueT *dst = data->mValues, *end = dst + SrcNode0::SIZE; dst != end; dst += 4, src += 4) {
                dst[0] = src[0]; // copy *all* voxel values in sets of four, i.e. loop-unrolling
                dst[1] = src[1];
                dst[2] = src[2];
                dst[3] = src[3];
            }
        }
    });
} // GridBuilder::processLeafs<T>

//================================================================================================

template<typename BuildT, typename StatsT>
template<typename T>
inline typename std::enable_if<is_same<ValueMask,typename T::BuildType>::value>::type
GridBuilder<BuildT, StatsT>::
    processLeafs(std::vector<T*>& srcLeafs)
{
    forEach(srcLeafs, 8, [&](const Range1D& r) {
        auto *ptr = mBufferPtr + mBufferOffsets[5];
        for (auto i = r.begin(); i != r.end(); ++i) {
            auto *srcLeaf = srcLeafs[i];
            auto *dstLeaf = PtrAdd<DstNode0>(ptr, srcLeaf->mDstOffset);
            auto *data = dstLeaf->data();
            if (DstNode0::DataType::padding()>0u) {
                std::memset(data, 0, DstNode0::DataType::memUsage());
            } else {
                data->mBBoxDif[0] = 0u;
                data->mBBoxDif[1] = 0u;
                data->mBBoxDif[2] = 0u;
                data->mFlags = 0u;// enable rendering, no bbox, no stats
                data->mPadding[0] = data->mPadding[1] = 0u;
            }
            srcLeaf->mDstNode = dstLeaf;
            data->mBBoxMin = srcLeaf->mOrigin; // copy origin of node
            data->mValueMask = srcLeaf->mValueMask; // copy value mask
        }
    });
} // GridBuilder::processLeafs<ValueMask>

//================================================================================================

template<typename BuildT, typename StatsT>
template<typename T>
inline typename std::enable_if<is_same<bool,typename T::BuildType>::value>::type
GridBuilder<BuildT, StatsT>::
    processLeafs(std::vector<T*>& srcLeafs)
{
    forEach(srcLeafs, 8, [&](const Range1D& r) {
        auto *ptr = mBufferPtr + mBufferOffsets[5];
        for (auto i = r.begin(); i != r.end(); ++i) {
            auto *srcLeaf = srcLeafs[i];
            auto *dstLeaf = PtrAdd<DstNode0>(ptr, srcLeaf->mDstOffset);
            auto *data = dstLeaf->data();
            if (DstNode0::DataType::padding()>0u) {
                std::memset(data, 0, DstNode0::DataType::memUsage());
            } else {
                data->mBBoxDif[0] = 0u;
                data->mBBoxDif[1] = 0u;
                data->mBBoxDif[2] = 0u;
                data->mFlags = 0u;// enable rendering, no bbox, no stats
            }
            srcLeaf->mDstNode = dstLeaf;
            data->mBBoxMin = srcLeaf->mOrigin; // copy origin of node
            data->mValueMask = srcLeaf->mValueMask; // copy value mask
            data->mValues = srcLeaf->mValues; // copy value mask
        }
    });
} // GridBuilder::processLeafs<bool>

//================================================================================================

template<typename BuildT, typename StatsT>
template<typename T>
inline typename std::enable_if<is_same<Fp4,  typename T::BuildType>::value ||
                               is_same<Fp8,  typename T::BuildType>::value ||
                               is_same<Fp16, typename T::BuildType>::value>::type
GridBuilder<BuildT, StatsT>::
    processLeafs(std::vector<T*>& srcLeafs)
{
    static_assert(is_same<float, ValueT>::value, "Expected ValueT == float");
    using ArrayT = typename DstNode0::DataType::ArrayType;
    using FloatT = typename std::conditional<DstNode0::DataType::bitWidth()>=16, double, float>::type;// 16 compression and higher requires double
    static constexpr FloatT UNITS = FloatT((1 << DstNode0::DataType::bitWidth()) - 1);// # of unique non-zero values
    DitherLUT lut(mDitherOn);

    forEach(srcLeafs, 8, [&](const Range1D& r) {
        uint8_t* ptr = mBufferPtr + mBufferOffsets[5];
        for (auto i = r.begin(); i != r.end(); ++i) {
            auto *srcLeaf = srcLeafs[i];
            auto *dstLeaf = PtrAdd<DstNode0>(ptr, srcLeaf->mDstOffset);
            srcLeaf->mDstNode = dstLeaf;
            auto *data = dstLeaf->data();
            if (DstNode0::DataType::padding()>0u) {
                std::memset(data, 0, DstNode0::DataType::memUsage());
            } else {
                data->mFlags = data->mBBoxDif[2] = data->mBBoxDif[1] = data->mBBoxDif[0] = 0u;
                data->mDev = data->mAvg = data->mMax = data->mMin = 0u;
            }
            data->mBBoxMin = srcLeaf->mOrigin; // copy origin of node
            data->mValueMask = srcLeaf->mValueMask; // copy value mask
            const float* src = srcLeaf->mValues;
            // compute extrema values
            float min = std::numeric_limits<float>::max(), max = -min;
            for (int i=0; i<512; ++i) {
                const float v = src[i];
                if (v < min) min = v;
                if (v > max) max = v;
            }
            data->init(min, max, DstNode0::DataType::bitWidth());
            // perform quantization relative to the values in the current leaf node
            const FloatT encode = UNITS/(max-min);
            auto *code = reinterpret_cast<ArrayT*>(data->mCode);
            int offset = 0;
            if (is_same<Fp4, BuildT>::value) {// resolved at compile-time
                for (int j=0; j<128; ++j) {
                    auto tmp = ArrayT(encode * (*src++ - min) + lut(offset++));
                    *code++  = ArrayT(encode * (*src++ - min) + lut(offset++)) << 4 | tmp;
                    tmp      = ArrayT(encode * (*src++ - min) + lut(offset++));
                    *code++  = ArrayT(encode * (*src++ - min) + lut(offset++)) << 4 | tmp;
                }
            } else {
                for (int j=0; j<128; ++j) {
                    *code++ = ArrayT(encode * (*src++ - min) + lut(offset++));
                    *code++ = ArrayT(encode * (*src++ - min) + lut(offset++));
                    *code++ = ArrayT(encode * (*src++ - min) + lut(offset++));
                    *code++ = ArrayT(encode * (*src++ - min) + lut(offset++));
                }
            }
        }
    });
} // GridBuilder::processLeafs<Fp4, Fp8, Fp16>

//================================================================================================

template<typename BuildT, typename StatsT>
template<typename T>
inline typename std::enable_if<is_same<FpN, typename T::BuildType>::value>::type
GridBuilder<BuildT, StatsT>::
    processLeafs(std::vector<T*>& srcLeafs)
{
    static_assert(is_same<float, ValueT>::value, "Expected ValueT == float");

    DitherLUT lut(mDitherOn);
    forEach(srcLeafs, 8, [&](const Range1D& r) {
        uint8_t* ptr = mBufferPtr + mBufferOffsets[5];
        for (auto i = r.begin(); i != r.end(); ++i) {
            auto *srcLeaf = srcLeafs[i];
            auto *dstLeaf = PtrAdd<DstNode0>(ptr, srcLeaf->mDstOffset);
            auto *data = dstLeaf->data();
            data->mBBoxMin = srcLeaf->mOrigin; // copy origin of node
            data->mBBoxDif[0] = 0u;
            data->mBBoxDif[1] = 0u;
            data->mBBoxDif[2] = 0u;
            srcLeaf->mDstNode = dstLeaf;
            const uint8_t logBitWidth = uint8_t(mCodec[i].log2);
            data->mFlags = logBitWidth << 5;// pack logBitWidth into 3 MSB of mFlag
            data->mValueMask = srcLeaf->mValueMask; // copy value mask
            const float* src = srcLeaf->mValues;
            const float min = mCodec[i].min, max = mCodec[i].max;
            data->init(min, max, uint8_t(1) << logBitWidth);
            // perform quantization relative to the values in the current leaf node
            int offset = 0;
            switch (logBitWidth) {
                case 0u: {// 1 bit
                    auto *dst = reinterpret_cast<uint8_t*>(data+1);
                    const float encode = 1.0f/(max - min);
                    for (int j=0; j<64; ++j) {
                        uint8_t a = 0;
                        for (int k=0; k<8; ++k) {
                            a |= uint8_t(encode * (*src++ - min) + lut(offset++)) << k;
                        }
                        *dst++ = a;
                    }
                }
                break;
                case 1u: {// 2 bits
                    auto *dst = reinterpret_cast<uint8_t*>(data+1);
                    const float encode = 3.0f/(max - min);
                    for (int j=0; j<128; ++j) {
                        auto a = uint8_t(encode * (*src++ - min) + lut(offset++));
                        a     |= uint8_t(encode * (*src++ - min) + lut(offset++)) << 2;
                        a     |= uint8_t(encode * (*src++ - min) + lut(offset++)) << 4;
                        *dst++ = uint8_t(encode * (*src++ - min) + lut(offset++)) << 6 | a;
                    }
                }
                break;
                case 2u: {// 4 bits
                    auto *dst = reinterpret_cast<uint8_t*>(data+1);
                    const float encode = 15.0f/(max - min);
                    for (int j=0; j<128; ++j) {
                        auto a = uint8_t(encode * (*src++ - min) + lut(offset++));
                        *dst++ = uint8_t(encode * (*src++ - min) + lut(offset++)) << 4 | a;
                        a      = uint8_t(encode * (*src++ - min) + lut(offset++));
                        *dst++ = uint8_t(encode * (*src++ - min) + lut(offset++)) << 4 | a;
                    }
                }
                break;
                case 3u: {// 8 bits
                    auto *dst = reinterpret_cast<uint8_t*>(data+1);
                    const float encode = 255.0f/(max - min);
                    for (int j=0; j<128; ++j) {
                        *dst++ = uint8_t(encode * (*src++ - min) + lut(offset++));
                        *dst++ = uint8_t(encode * (*src++ - min) + lut(offset++));
                        *dst++ = uint8_t(encode * (*src++ - min) + lut(offset++));
                        *dst++ = uint8_t(encode * (*src++ - min) + lut(offset++));
                    }
                }
                break;
                default: {// 16 bits
                    auto *dst = reinterpret_cast<uint16_t*>(data+1);
                    const double encode = 65535.0/(max - min);// note that double is required!
                    for (int j=0; j<128; ++j) {
                        *dst++ = uint16_t(encode * (*src++ - min) + lut(offset++));
                        *dst++ = uint16_t(encode * (*src++ - min) + lut(offset++));
                        *dst++ = uint16_t(encode * (*src++ - min) + lut(offset++));
                        *dst++ = uint16_t(encode * (*src++ - min) + lut(offset++));
                    }
                }
            }// end switch
        }
    });// kernel
} // GridBuilder::processLeafs<FpN>

//================================================================================================

template<typename BuildT, typename StatsT>
template<typename SrcNodeT>
void GridBuilder<BuildT, StatsT>::
    processNodes(std::vector<SrcNodeT*>& srcNodes)
{
    using DstNodeT = typename SrcNodeT::NanoNodeT;
    static_assert(DstNodeT::LEVEL == 1 || DstNodeT::LEVEL == 2, "Expected internal node");
    forEach(srcNodes, 4, [&](const Range1D& r) {
        uint8_t* ptr = mBufferPtr + mBufferOffsets[5 - DstNodeT::LEVEL];// 3 or 4
        for (auto i = r.begin(); i != r.end(); ++i) {
            SrcNodeT *srcNode = srcNodes[i];
            DstNodeT *dstNode = PtrAdd<DstNodeT>(ptr, srcNode->mDstOffset);
            auto     *data = dstNode->data();
            if (DstNodeT::DataType::padding()>0u) std::memset(data, 0, DstNodeT::memUsage());
            srcNode->mDstNode = dstNode;
            data->mBBox[0]   = srcNode->mOrigin; // copy origin of node
            data->mValueMask = srcNode->mValueMask; // copy value mask
            data->mChildMask = srcNode->mChildMask; // copy child mask
            for (uint32_t j = 0; j != SrcNodeT::SIZE; ++j) {
                if (data->mChildMask.isOn(j)) {
                    data->setChild(j, srcNode->mTable[j].child->mDstNode);
                } else
                    data->setValue(j, srcNode->mTable[j].value);
            }
        }
    });
} // GridBuilder::processNodes

//================================================================================================

template<typename BuildT, typename StatsT>
NanoRoot<BuildT>* GridBuilder<BuildT, StatsT>::processRoot()
{
    auto *dstRoot = reinterpret_cast<DstRootT*>(mBufferPtr + mBufferOffsets[2]);
    auto *data = dstRoot->data();
    if (data->padding()>0) std::memset(data, 0, DstRootT::memUsage(uint32_t(mRoot.mTable.size())));
    data->mTableSize = uint32_t(mRoot.mTable.size());
    data->mMinimum = data->mMaximum = data->mBackground = mRoot.mBackground;
    data->mBBox = CoordBBox(); // // set to an empty bounding box

    uint32_t tileID = 0;
    for (auto iter = mRoot.mTable.begin(); iter != mRoot.mTable.end(); ++iter) {
        auto *dstTile = data->tile(tileID++);
        if (auto* srcChild = iter->second.child) {
            dstTile->setChild(srcChild->mOrigin, srcChild->mDstNode, data);
        } else {
            dstTile->setValue(iter->first, iter->second.state, iter->second.value);
        }
    }
    return dstRoot;
} // GridBuilder::processRoot

//================================================================================================

template<typename BuildT, typename StatsT>
NanoTree<BuildT>* GridBuilder<BuildT, StatsT>::processTree()
{
    auto *dstTree = reinterpret_cast<DstTreeT*>(mBufferPtr + mBufferOffsets[1]);
    auto *data = dstTree->data();
    data->setRoot( this->processRoot() );

    DstNode2 *node2 = mArray2.empty() ? nullptr : reinterpret_cast<DstNode2*>(mBufferPtr + mBufferOffsets[3]);
    data->setFirstNode(node2);

    DstNode1 *node1 = mArray1.empty() ? nullptr : reinterpret_cast<DstNode1*>(mBufferPtr + mBufferOffsets[4]);
    data->setFirstNode(node1);

    DstNode0 *node0 = mArray0.empty() ? nullptr : reinterpret_cast<DstNode0*>(mBufferPtr + mBufferOffsets[5]);
    data->setFirstNode(node0);

    data->mNodeCount[0] = static_cast<uint32_t>(mArray0.size());
    data->mNodeCount[1] = static_cast<uint32_t>(mArray1.size());
    data->mNodeCount[2] = static_cast<uint32_t>(mArray2.size());

    // Count number of active leaf level tiles
    data->mTileCount[0] = reduce(mArray1, uint32_t(0), [&](Range1D &r, uint32_t sum){
        for (auto i=r.begin(); i!=r.end(); ++i) sum += mArray1[i]->mValueMask.countOn();
        return sum;}, std::plus<uint32_t>());

    // Count number of active lower internal node tiles
    data->mTileCount[1] = reduce(mArray2, uint32_t(0), [&](Range1D &r, uint32_t sum){
        for (auto i=r.begin(); i!=r.end(); ++i) sum += mArray2[i]->mValueMask.countOn();
        return sum;}, std::plus<uint32_t>());

    // Count number of active upper internal node tiles
    uint32_t sum = 0;
    for (auto &tile : mRoot.mTable) {
        if (tile.second.child==nullptr && tile.second.state) ++sum;
    }
    data->mTileCount[2] = sum;

    // Count number of active voxels
    data->mVoxelCount = reduce(mArray0, uint64_t(0), [&](Range1D &r, uint64_t sum){
        for (auto i=r.begin(); i!=r.end(); ++i) sum += mArray0[i]->mValueMask.countOn();
        return sum;}, std::plus<uint64_t>());

    data->mVoxelCount += data->mTileCount[0]*DstNode0::NUM_VALUES;
    data->mVoxelCount += data->mTileCount[1]*DstNode1::NUM_VALUES;
    data->mVoxelCount += data->mTileCount[2]*DstNode2::NUM_VALUES;

    return dstTree;
} // GridBuilder::processTree

//================================================================================================

template<typename BuildT, typename StatsT>
NanoGrid<BuildT>* GridBuilder<BuildT, StatsT>::
processGrid(const Map&         map,
            const std::string& name)
{
    auto *dstGrid = reinterpret_cast<DstGridT*>(mBufferPtr + mBufferOffsets[0]);
    this->processTree();
    auto* data = dstGrid->data();
    data->mMagic = NANOVDB_MAGIC_NUMBER;
    data->mChecksum = 0u;
    data->mVersion = Version();
    data->mFlags = static_cast<uint32_t>(GridFlags::IsBreadthFirst);
    data->mGridIndex = 0;
    data->mGridCount = 1;
    data->mGridSize = mBufferOffsets[8];
    data->mWorldBBox = BBox<Vec3R>();
    data->mBlindMetadataOffset = 0;
    data->mBlindMetadataCount = 0;
    data->mGridClass = mGridClass;
    data->mGridType = mapToGridType<BuildT>();
    data->mData0 = 0u;
    data->mData1 = 0u;
    data->mData2 = 0u;

    if (!isValid(data->mGridType, data->mGridClass)) {
        std::stringstream ss;
        ss << "Invalid combination of GridType("<<int(data->mGridType)
           << ") and GridClass("<<int(data->mGridClass)<<"). See NanoVDB.h for details!";
        throw std::runtime_error(ss.str());
    }

    std::memset(data->mGridName, '\0', GridData::MaxNameSize);//overwrite mGridName
    strncpy(data->mGridName, name.c_str(), GridData::MaxNameSize-1);
    if (name.length() >= GridData::MaxNameSize) {//  currently we don't support long grid names
        std::stringstream ss;
        ss << "Grid name \"" << name << "\" is more then " << GridData::MaxNameSize << " characters";
        throw std::runtime_error(ss.str());
    }

    data->mVoxelSize = map.applyMap(Vec3d(1)) - map.applyMap(Vec3d(0));
    data->mMap = map;

    if (mBlindDataSize>0) {
        auto *metaData = reinterpret_cast<GridBlindMetaData*>(mBufferPtr + mBufferOffsets[6]);
        data->mBlindMetadataOffset = PtrDiff(metaData, dstGrid);
        data->mBlindMetadataCount = 1u;// we currently support only 1 set of blind data
        auto *blindData = reinterpret_cast<char*>(mBufferPtr + mBufferOffsets[7]);
        metaData->setBlindData(blindData);
    }

    return dstGrid;
} // GridBuilder::processGrid

//================================================================================================

} // namespace nanovdb

#endif // NANOVDB_GRIDBUILDER_H_HAS_BEEN_INCLUDED
