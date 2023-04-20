// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

/*!
    \file IndexGridBuilder.h

    \author Ken Museth

    \date July 8, 2022

    \brief Generates a NanoVDB IndexGrid or OnIndexGrid from any existing NanoVDB grid.
           An IndexGrid will index both active and inactive values in the input grid
           whereas an OnIndexGrid will only index active values. Both grids can optionally
           index stats and tile values.

    \note An IndexGrid encodes index offsets to external value arrays
*/

#ifndef NANOVDB_INDEXGRIDBUILDER_H_HAS_BEEN_INCLUDED
#define NANOVDB_INDEXGRIDBUILDER_H_HAS_BEEN_INCLUDED

#include "GridHandle.h"
#include "NodeManager.h"
#include "Range.h"
#include "ForEach.h"

#include <map>
#include <limits>
#include <iostream>
#include <sstream> // for stringstream
#include <vector>
#include <cstring> // for memcpy

namespace nanovdb {

/// @brief Allows for the construction of NanoVDB grids without any dependency
template <typename SrcValueT, typename DstValueT = ValueIndex>
class IndexGridBuilder
{
    using SrcNode0 = NanoLeaf< SrcValueT>;
    using SrcNode1 = NanoLower<SrcValueT>;
    using SrcNode2 = NanoUpper<SrcValueT>;
    using SrcData0 = typename SrcNode0::DataType;
    using SrcData1 = typename SrcNode1::DataType;
    using SrcData2 = typename SrcNode2::DataType;
    using SrcRootT = NanoRoot<SrcValueT>;
    using SrcTreeT = NanoTree<SrcValueT>;
    using SrcGridT = NanoGrid<SrcValueT>;

    static constexpr bool IsSparse = is_same<DstValueT, ValueOnIndex>::value;
    static_assert(is_same<DstValueT, ValueIndex>::value || IsSparse, "Expected ValueIndex or ValueOnIndex");
    using DstNode0 = NanoLeaf< DstValueT>;
    using DstNode1 = NanoLower<DstValueT>;
    using DstNode2 = NanoUpper<DstValueT>;
    using DstData0 = typename NanoLeaf< DstValueT>::DataType;
    using DstData1 = typename NanoLower<DstValueT>::DataType;
    using DstData2 = typename NanoUpper<DstValueT>::DataType;
    using DstRootT = NanoRoot< DstValueT>;
    using DstTreeT = NanoTree< DstValueT>;
    using DstGridT = NanoGrid< DstValueT>;

    NodeManagerHandle<>    mSrcMgrHandle;
    NodeManager<SrcValueT> *mSrcMgr;
    std::vector<uint64_t>  mValIdx2, mValIdx1, mValIdx0;// store id of first value in node
    uint8_t*               mBufferPtr;// pointer to the beginning of the buffer
    uint64_t               mBufferOffsets[9];//grid, tree, root, upper, lower, leafs, meta, data, buffer size
    uint64_t               mValueCount;// total number of values indexed
    const bool             mIncludeStats, mIncludeTiles;// include stats and tile values

    DstNode0* getLeaf( int i=0) const {return PtrAdd<DstNode0>(mBufferPtr, mBufferOffsets[5]) + i;}
    DstNode1* getLower(int i=0) const {return PtrAdd<DstNode1>(mBufferPtr, mBufferOffsets[4]) + i;}
    DstNode2* getUpper(int i=0) const {return PtrAdd<DstNode2>(mBufferPtr, mBufferOffsets[3]) + i;}
    DstRootT* getRoot() const {return PtrAdd<DstRootT>(mBufferPtr, mBufferOffsets[2]);}
    DstTreeT* getTree() const {return PtrAdd<DstTreeT>(mBufferPtr, mBufferOffsets[1]);}
    DstGridT* getGrid() const {return PtrAdd<DstGridT>(mBufferPtr, mBufferOffsets[0]);}

    // Count the number of values (possibly only active)
    void countValues();

    // Below are private methods use to serialize nodes into NanoVDB
    template<typename BufferT>
    GridHandle<BufferT> initHandle(uint32_t channels, const BufferT& buffer);

    void processLeafs();

    void processLower();

    void processUpper();

    void processRoot();

    void processTree();

    void processGrid(const std::string& name, uint32_t channels);

    template<typename T = SrcValueT>
    typename std::enable_if<!is_same<ValueMask, T>::value>::type
    processChannels(uint32_t channels);

    template<typename T = SrcValueT>
    typename std::enable_if<is_same<ValueMask, T>::value>::type
    processChannels(uint32_t) {}// since ValueMask has no values this is a no-op

public:

    /// @brief Constructor based on a source grid
    ///
    /// @param srcGrid         Source grid used to generate the IndexGrid.
    /// @param includeStats    Include indices for min/max/avg/std per node.
    /// @param includeTiles    Include indices for tiles, i.e. non-leaf-values.
    ///
    /// @note For minimum memory consumption set the two boolean options to false
    IndexGridBuilder(const SrcGridT& srcGrid, bool includeStats = true, bool includeTiles = true)
        : mSrcMgrHandle(createNodeManager(srcGrid))
        , mSrcMgr(mSrcMgrHandle.template mgr<SrcValueT>())
        , mValueCount(0)
        , mIncludeStats(includeStats)
        , mIncludeTiles(includeTiles)
    {}

    /// @brief Return an instance of a GridHandle (invoking move semantics)
    ///
    /// @param name      String name of the return IndexGrid
    /// @param channels  Number of channels to be copied as blind data
    /// @param buffer    Buffer used to allocate the return handle for the IndexGrid
    ///
    template<typename BufferT = HostBuffer>
    GridHandle<BufferT> getHandle(const std::string& name = "", uint32_t channels = 0u, const BufferT& buffer = BufferT());

    /// @brief return the total number of values located in the source grid.
    ///
    /// @note This is minimum number of elements required for the external array that the IndexGrid
    ///       points to.
    uint64_t getValueCount() const { return mValueCount; }

    /// @brief return a buffer with all the values in the source grid
    template<typename BufferT = HostBuffer, typename T = SrcValueT>
    typename std::enable_if<!is_same<ValueMask, T>::value, BufferT>::type
    getValues(uint32_t channels = 1u, const BufferT &buffer = BufferT());

    /// @brief copy values from the source grid into the provided array and returns number of values copied
    template<typename T>
    typename std::enable_if<!is_same<ValueMask, T>::value, uint64_t>::type
    copyValues(T *buffer, size_t maxValueCount = -1);
}; // IndexGridBuilder

//================================================================================================

template<typename SrcValueT, typename DstValueT>
template<typename BufferT>
GridHandle<BufferT> IndexGridBuilder<SrcValueT, DstValueT>::
getHandle(const std::string &name, uint32_t channels, const BufferT &buffer)
{
    this->countValues();

    auto handle = this->template initHandle<BufferT>(channels, buffer);// initialize the arrays of nodes

    this->processLeafs();

    this->processLower();

    this->processUpper();

    this->processRoot();

    this->processTree();

    this->processGrid(name, channels);

    this->processChannels(channels);

    return handle;
} // IndexGridBuilder::getHandle

//================================================================================================

template<typename SrcValueT, typename DstValueT>
void IndexGridBuilder<SrcValueT, DstValueT>::countValues()
{
    const uint64_t stats = mIncludeStats ? 4u : 0u;// minimum, maximum, average, and deviation
    uint64_t valueCount = 1u;// offset 0 corresponds to the background value

    if (mIncludeTiles) {
        // root values
        if (IsSparse) {
            for (auto it = mSrcMgr->root().beginValueOn(); it; ++it) ++valueCount;
        } else {
            for (auto it = mSrcMgr->root().beginValue(); it; ++it) ++valueCount;
        }
        valueCount += stats;// optionally append stats for the root node

        // tile values in upper internal nodes
        mValIdx2.resize(mSrcMgr->nodeCount(2) + 1, stats);
        if (IsSparse) {
            forEach(1, mValIdx2.size(), 8, [&](const Range1D& r){
                for (auto i = r.begin(); i!=r.end(); ++i) {
                    mValIdx2[i] += mSrcMgr->upper(i-1).data()->mValueMask.countOn();
                }
            });
        } else {
            forEach(1, mValIdx2.size(), 8, [&](const Range1D& r){
                for (auto i = r.begin(); i!=r.end(); ++i) {
                    mValIdx2[i] += 32768u - mSrcMgr->upper(i-1).data()->mChildMask.countOn();
                }
            });
        }
        mValIdx2[0] = valueCount;
        for (size_t i=1; i<mValIdx2.size(); ++i) mValIdx2[i] += mValIdx2[i-1];// pre-fixed sum
        valueCount = mValIdx2.back();

        // tile values in lower internal nodes
        mValIdx1.resize(mSrcMgr->nodeCount(1) + 1, stats);
        if (IsSparse) {
            forEach(1, mValIdx1.size(), 8, [&](const Range1D& r){
                for (auto i = r.begin(); i!=r.end(); ++i) {
                    mValIdx1[i] += mSrcMgr->lower(i-1).data()->mValueMask.countOn();
                }
            });
        } else {
            forEach(1, mValIdx1.size(), 8, [&](const Range1D& r){
                for (auto i = r.begin(); i!=r.end(); ++i) {
                    mValIdx1[i] += 4096u - mSrcMgr->lower(i-1).data()->mChildMask.countOn();
                }
            });
        }
        mValIdx1[0] = valueCount;
        for (size_t i=1; i<mValIdx1.size(); ++i) mValIdx1[i] += mValIdx1[i-1];// pre-fixed sum
        valueCount = mValIdx1.back();
    }// if (mIncludeTiles)

    // voxel values in leaf nodes
    mValIdx0.clear();
    mValIdx0.resize(mSrcMgr->nodeCount(0) + 1, 512u + stats);
    if (IsSparse) {
        forEach(1, mValIdx0.size(), 8, [&](const Range1D& r) {
            for (auto i = r.begin(); i != r.end(); ++i) {
                mValIdx0[i] = stats + mSrcMgr->leaf(i-1).data()->mValueMask.countOn();
            }
        });
    }
    mValIdx0[0] = valueCount;
    for (size_t i=1; i<mValIdx0.size(); ++i) mValIdx0[i] += mValIdx0[i-1];// pre-fixed sum

    mValueCount = mValIdx0.back();
}// countValues


//================================================================================================
template<typename SrcValueT, typename DstValueT>
template<typename T>
typename std::enable_if<!is_same<ValueMask, T>::value, uint64_t>::type
IndexGridBuilder<SrcValueT, DstValueT>::copyValues(T *buffer, size_t maxValueCount)
{// copy values from the source grid into the provided buffer and return the number of values copied
    assert(mBufferPtr);
    if (maxValueCount < mValueCount) return 0;

    // Value array always starts with the background value
    buffer[0] = mSrcMgr->root().background();

    if (mIncludeTiles) {
        // copy root tile values and optionally stats
        auto *srcData = mSrcMgr->root().data();
        SrcValueT *v = buffer + 1u;
        for (uint32_t tileID = 0; tileID < srcData->mTableSize; ++tileID) {
            auto *srcTile = srcData->tile(tileID);
            if (srcTile->isChild() || (IsSparse && !srcTile->state)) continue;
            NANOVDB_ASSERT(v - buffer < mValueCount);
            *v++ = srcTile->value;
        }
        if (mIncludeStats) {
            *v++ = mSrcMgr->root().minimum();
            *v++ = mSrcMgr->root().maximum();
            *v++ = mSrcMgr->root().average();
            *v++ = mSrcMgr->root().stdDeviation();
        }

        // upper nodes
        forEach(0, mSrcMgr->nodeCount(2), 1, [&](const Range1D& r) {
            DstData2 *dstData = this->getUpper(r.begin())->data();
            for (auto i = r.begin(); i != r.end(); ++i, ++dstData) {
                SrcValueT *v = buffer + mValIdx2[i];
                const SrcNode2 &srcNode = mSrcMgr->upper(i);
                if (IsSparse) {
                    for (auto it = srcNode.beginValueOn(); it; ++it) *v++ = *it;
                } else {
                    auto *srcData = srcNode.data();
                    for (uint32_t j = 0u; j != 32768u; ++j) {
                        if (srcData->mChildMask.isOn(j)) continue;
                        *v++ = srcData->getValue(j);
                    }
                }
                if (mIncludeStats) {
                    *v++ = srcNode.minimum();
                    *v++ = srcNode.maximum();
                    *v++ = srcNode.average();
                    *v++ = srcNode.stdDeviation();
                }
                NANOVDB_ASSERT(v - buffer < mValueCount);
            }
        });

        // lower nodes
        forEach(0, mSrcMgr->nodeCount(1), 4, [&](const Range1D& r) {
            DstData1 *dstData = this->getLower(r.begin())->data();
            for (auto i = r.begin(); i != r.end(); ++i, ++dstData) {
                SrcValueT *v = buffer + mValIdx1[i];
                const SrcNode1 &srcNode = mSrcMgr->lower(i);
                if (IsSparse) {
                    for (auto it = srcNode.beginValueOn(); it; ++it) *v++ = *it;
                } else {
                    auto *srcData = srcNode.data();
                    for (uint32_t j = 0u; j != 4096u; ++j) {
                        if (srcData->mChildMask.isOn(j)) continue;
                        *v++ = srcData->getValue(j);
                    }
                }
                if (mIncludeStats) {
                    *v++ = srcNode.minimum();
                    *v++ = srcNode.maximum();
                    *v++ = srcNode.average();
                    *v++ = srcNode.stdDeviation();
                }
            }
        });
    }// if (mIncludeTiles)

    // leaf nodes
    forEach(0, mSrcMgr->nodeCount(0), 8, [&](const Range1D& r) {
        DstData0 *dstLeaf = this->getLeaf(r.begin())->data();
        for (auto i = r.begin(); i != r.end(); ++i, ++dstLeaf) {
            SrcValueT *v = buffer + mValIdx0[i];
            const SrcNode0 &srcLeaf = mSrcMgr->leaf(i);
            if (IsSparse) {
                for (auto it = srcLeaf.beginValueOn(); it; ++it) *v++ = *it;
            } else {
                const SrcData0 *srcData = srcLeaf.data();
                for (uint32_t j = 0u; j != 512u; ++j) *v++ = srcData->getValue(j);
            }
            if (mIncludeStats) {
                *v++ = srcLeaf.minimum();
                *v++ = srcLeaf.maximum();
                *v++ = srcLeaf.average();
                *v++ = srcLeaf.stdDeviation();
            }
        }
    });

    return mValueCount;
} // IndexGridBuilder::copyValues

template<typename SrcValueT, typename DstValueT>
template<typename BufferT, typename T>
typename std::enable_if<!is_same<ValueMask, T>::value, BufferT>::type
IndexGridBuilder<SrcValueT, DstValueT>::getValues(uint32_t channels, const BufferT &buffer)
{
    assert(channels > 0);
    auto values = BufferT::create(channels*sizeof(SrcValueT)*mValueCount, &buffer);
    SrcValueT *p = reinterpret_cast<SrcValueT*>(values.data());
    if (!this->copyValues(p, mValueCount)) {
        throw std::runtime_error("getValues: insufficient channels");
    }
    for (uint32_t i=1; i<channels; ++i) {
        nanovdb::forEach(0,mValueCount,1024,[&](const nanovdb::Range1D &r){
            SrcValueT *dst=p+i*mValueCount+r.begin(), *end=dst+r.size(), *src=dst-mValueCount;
            while(dst!=end) *dst++ = *src++;
        });
    }
    return values;
} // IndexGridBuilder::getValues

//================================================================================================

template<typename SrcValueT, typename DstValueT>
template<typename BufferT>
GridHandle<BufferT> IndexGridBuilder<SrcValueT, DstValueT>::
initHandle(uint32_t channels, const BufferT& buffer)
{
    const SrcTreeT &srcTree = mSrcMgr->tree();
    mBufferOffsets[0] = 0;// grid is always stored at the start of the buffer!
    mBufferOffsets[1] = DstGridT::memUsage(); // tree
    mBufferOffsets[2] = mBufferOffsets[1] + DstTreeT::memUsage(); // root
    mBufferOffsets[3] = mBufferOffsets[2] + DstRootT::memUsage(srcTree.root().tileCount());// upper internal nodes
    mBufferOffsets[4] = mBufferOffsets[3] + srcTree.nodeCount(2)*sizeof(DstData2); // lower internal nodes
    mBufferOffsets[5] = mBufferOffsets[4] + srcTree.nodeCount(1)*sizeof(DstData1); // leaf nodes
    mBufferOffsets[6] = mBufferOffsets[5] + srcTree.nodeCount(0)*sizeof(DstData0); // meta data
    mBufferOffsets[7] = mBufferOffsets[6] + GridBlindMetaData::memUsage(channels); // channel values
    mBufferOffsets[8] = mBufferOffsets[7] + channels*mValueCount*sizeof(SrcValueT);// total size

    GridHandle<BufferT> handle(BufferT::create(mBufferOffsets[8], &buffer));
    mBufferPtr = handle.data();

    return handle;
} // IndexGridBuilder::initHandle

//================================================================================================

template<typename SrcValueT, typename DstValueT>
void IndexGridBuilder<SrcValueT, DstValueT>::processGrid(const std::string& name, uint32_t channels)
{
    auto *srcData = mSrcMgr->grid().data();
    auto *dstData = this->getGrid()->data();

    dstData->mMagic = NANOVDB_MAGIC_NUMBER;
    dstData->mChecksum = 0u;
    dstData->mVersion = Version();
    dstData->mFlags = static_cast<uint32_t>(GridFlags::IsBreadthFirst);
    dstData->mGridIndex = 0;
    dstData->mGridCount = 1;
    dstData->mGridSize = mBufferOffsets[8];
    std::memset(dstData->mGridName, '\0', GridData::MaxNameSize);//overwrite mGridName
    strncpy(dstData->mGridName, name.c_str(), GridData::MaxNameSize-1);
    dstData->mMap = srcData->mMap;
    dstData->mWorldBBox = srcData->mWorldBBox;
    dstData->mVoxelSize = srcData->mVoxelSize;
    dstData->mGridClass = GridClass::IndexGrid;
    dstData->mGridType = mapToGridType<DstValueT>();
    dstData->mBlindMetadataOffset = mBufferOffsets[6];
    dstData->mBlindMetadataCount = channels;
    dstData->mData0 = 0u;
    dstData->mData1 = mValueCount;// encode the total number of values being indexed
    dstData->mData2 = 0u;

    if (name.length() >= GridData::MaxNameSize) {//  currently we don't support long grid names
        std::stringstream ss;
        ss << "Grid name \"" << name << "\" is more then " << GridData::MaxNameSize << " characters";
        throw std::runtime_error(ss.str());
    }
} // IndexGridBuilder::processGrid

//================================================================================================

template<typename SrcValueT, typename DstValueT>
void IndexGridBuilder<SrcValueT, DstValueT>::processTree()
{
    auto *srcData = mSrcMgr->tree().data();
    auto *dstData = this->getTree()->data();
    for (int i=0; i<4; ++i) dstData->mNodeOffset[i] = mBufferOffsets[5-i] - mBufferOffsets[1];// byte offset from tree to first leaf, lower, upper and root node
    for (int i=0; i<3; ++i) {
        dstData->mNodeCount[i] = srcData->mNodeCount[i];// total number of nodes of type: leaf, lower internal, upper internal
        dstData->mTileCount[i] = srcData->mTileCount[i];// total number of active tile values at the lower internal, upper internal and root node levels
    }
    dstData->mVoxelCount = srcData->mVoxelCount;// total number of active voxels in the root and all its child nodes
} // IndexGridBuilder::processTree

//================================================================================================

template<typename SrcValueT, typename DstValueT>
void IndexGridBuilder<SrcValueT, DstValueT>::processRoot()
{
    auto *srcData = mSrcMgr->root().data();
    auto *dstData = this->getRoot()->data();

    if (dstData->padding()>0) std::memset(dstData, 0, DstRootT::memUsage(mSrcMgr->root().tileCount()));
    dstData->mBBox = srcData->mBBox;
    dstData->mTableSize = srcData->mTableSize;
    dstData->mBackground = 0u;
    uint64_t valueCount = 0u;// the first entry is always the background value

    for (uint32_t tileID = 0, childID = 0; tileID < dstData->mTableSize; ++tileID) {
        auto *srcTile = srcData->tile(tileID);
        auto *dstTile = dstData->tile(tileID);
        dstTile->key = srcTile->key;
        if (srcTile->isChild()) {
            dstTile->child = childID * sizeof(DstNode2) + mBufferOffsets[3] - mBufferOffsets[2];
            dstTile->state = false;
            dstTile->value = std::numeric_limits<uint64_t>::max();
            ++childID;
        } else {
            dstTile->child = 0;
            dstTile->state = srcTile->state;
            dstTile->value = 0u;
            if (mIncludeTiles && !(IsSparse && !dstTile->state)) dstTile->value = ++valueCount;
        }
    }
    if (mIncludeTiles && mIncludeStats) {
        dstData->mMinimum = ++valueCount;
        dstData->mMaximum = ++valueCount;
        dstData->mAverage = ++valueCount;
        dstData->mStdDevi = ++valueCount;
    } else if (dstData->padding()==0) {
        dstData->mMinimum = 0u;
        dstData->mMaximum = 0u;
        dstData->mAverage = 0u;
        dstData->mStdDevi = 0u;
    }
} // IndexGridBuilder::processRoot

//================================================================================================

template<typename SrcValueT, typename DstValueT>
void IndexGridBuilder<SrcValueT, DstValueT>::processUpper()
{
    static_assert(DstData2::padding()==0u, "Expected upper internal nodes to have no padding");
    auto kernel = [&](const Range1D& r) {
        const bool hasStats = mIncludeStats;
        auto *dstData1 = this->getLower()->data();// fixed size
        auto *dstData2 = this->getUpper(r.begin())->data();// fixed size
        for (auto i = r.begin(); i != r.end(); ++i, ++dstData2) {
            SrcData2 *srcData2 = mSrcMgr->upper(i).data();// might vary in size due to compression
            dstData2->mBBox  = srcData2->mBBox;
            dstData2->mFlags = srcData2->mFlags;
            srcData2->mFlags = i;// encode node ID
            dstData2->mChildMask = srcData2->mChildMask;
            dstData2->mValueMask = srcData2->mValueMask;
            uint64_t n = mIncludeTiles ? mValIdx2[i] : 0u;
            for (uint32_t j = 0u; j != 32768u; ++j) {
                if (dstData2->isChild(j)) {
                    SrcData1 *srcChild = srcData2->getChild(j)->data();
                    DstData1 *dstChild = dstData1 + srcChild->mFlags;
                    dstData2->setChild(j, dstChild);
                    srcChild->mFlags = dstChild->mFlags;// restore
                } else {
                    uint64_t m = 0u;
                    if (mIncludeTiles && !(IsSparse && srcData2->mValueMask.isOff(j))) m = n++;
                    dstData2->setValue(j, m);
                }
            }
            if (mIncludeTiles && mIncludeStats) {
                dstData2->mMinimum = n++;
                dstData2->mMaximum = n++;
                dstData2->mAverage = n++;
                dstData2->mStdDevi = n++;
            } else {
                dstData2->mMinimum = 0u;
                dstData2->mMaximum = 0u;
                dstData2->mAverage = 0u;
                dstData2->mStdDevi = 0u;
            }
        }
    };
    forEach(0, mSrcMgr->nodeCount(2), 1, kernel);
} // IndexGridBuilder::processUpper

//================================================================================================

template<typename SrcValueT, typename DstValueT>
void IndexGridBuilder<SrcValueT, DstValueT>::processLower()
{
    static_assert(DstData1::padding()==0u, "Expected lower internal nodes to have no padding");
    auto kernel = [&](const Range1D& r) {
        DstData0 *dstData0 = this->getLeaf()->data();// first dst leaf node
        DstData1 *dstData1 = this->getLower(r.begin())->data();// fixed size
        for (auto i = r.begin(); i != r.end(); ++i, ++dstData1) {
            SrcData1 *srcData1 = mSrcMgr->lower(i).data();// might vary in size due to compression
            dstData1->mBBox = srcData1->mBBox;
            dstData1->mFlags = srcData1->mFlags;
            srcData1->mFlags = i;// encode node ID
            dstData1->mChildMask = srcData1->mChildMask;
            dstData1->mValueMask = srcData1->mValueMask;
            uint64_t n = mIncludeTiles ? mValIdx1[i] : 0u;
            for (uint32_t j = 0u; j != 4096u; ++j) {
                if (dstData1->isChild(j)) {
                    SrcData0 *srcChild = srcData1->getChild(j)->data();
                    DstData0 *dstChild = dstData0 + srcChild->mBBoxMin[0];
                    dstData1->setChild(j, dstChild);
                    srcChild->mBBoxMin[0] = dstChild->mBBoxMin[0];// restore
                } else {
                    uint64_t m = 0u;
                    if (mIncludeTiles && !(IsSparse && srcData1->mValueMask.isOff(j))) m = n++;
                    dstData1->setValue(j, m);
                }
            }
            if (mIncludeTiles && mIncludeStats) {
                dstData1->mMinimum = n++;
                dstData1->mMaximum = n++;
                dstData1->mAverage = n++;
                dstData1->mStdDevi = n++;
            } else {
                dstData1->mMinimum = 0u;
                dstData1->mMaximum = 0u;
                dstData1->mAverage = 0u;
                dstData1->mStdDevi = 0u;
            }
        }
    };
    forEach(0, mSrcMgr->nodeCount(1), 4, kernel);
} // IndexGridBuilder::processLower

//================================================================================================

template<typename SrcValueT, typename DstValueT>
void IndexGridBuilder<SrcValueT, DstValueT>::processLeafs()
{
    static_assert(DstData0::padding()==0u, "Expected leaf nodes to have no padding");

    auto kernel = [&](const Range1D& r) {
        DstData0 *dstData0 = this->getLeaf(r.begin())->data();// fixed size
        const uint8_t flags = mIncludeStats ? 16u : 0u;// 4th bit indicates stats
        for (auto i = r.begin(); i != r.end(); ++i, ++dstData0) {
            SrcData0 *srcData0 = mSrcMgr->leaf(i).data();// might vary in size due to compression
            dstData0->mBBoxMin = srcData0->mBBoxMin;
            srcData0->mBBoxMin[0] = int(i);// encode node ID
            dstData0->mBBoxDif[0] = srcData0->mBBoxDif[0];
            dstData0->mBBoxDif[1] = srcData0->mBBoxDif[1];
            dstData0->mBBoxDif[2] = srcData0->mBBoxDif[2];
            dstData0->mFlags = flags | (srcData0->mFlags & 2u);// 2nd bit indicates a bbox
            dstData0->mValueMask = srcData0->mValueMask;
            dstData0->mOffset = mValIdx0[i];
            dstData0->mPadding = 0u;
            if (IsSparse) {
                uint8_t *p = reinterpret_cast<uint8_t*>(&dstData0->mPadding), *q = p + 7;
                const uint64_t *w = dstData0->mValueMask.template getWords<uint64_t>();
                uint32_t sum = CountOn(*w++);
                for (int j=0; j<7; ++j) {
                    *p++ = sum & 255u;
                    *q |= (sum >> 8) << j;
                    sum += CountOn(*w++);
                }
            }
        }
    };
    forEach(0, mSrcMgr->nodeCount(0), 8, kernel);
} // IndexGridBuilder::processLeafs

//================================================================================================

template<typename SrcValueT, typename DstValueT>
template<typename T>
typename std::enable_if<!is_same<ValueMask, T>::value>::type
IndexGridBuilder<SrcValueT, DstValueT>::processChannels(uint32_t channels)
{
    for (uint32_t i=0; i<channels; ++i) {
        auto *metaData  = PtrAdd<GridBlindMetaData>(mBufferPtr, mBufferOffsets[6]) + i;
        auto *blindData = PtrAdd<SrcValueT>(mBufferPtr, mBufferOffsets[7]) + i*mValueCount;
        metaData->setBlindData(blindData);
        metaData->mElementCount = mValueCount;
        metaData->mFlags = 0;
        metaData->mSemantic  = GridBlindDataSemantic::Unknown;
        metaData->mDataClass = GridBlindDataClass::ChannelArray;
        metaData->mDataType  = mapToGridType<SrcValueT>();
        std::memset(metaData->mName, '\0', GridBlindMetaData::MaxNameSize);
        std::stringstream ss;
        ss << toStr(metaData->mDataType) << "_channel_" << i;
        strncpy(metaData->mName, ss.str().c_str(), GridBlindMetaData::MaxNameSize-1);
        if (i) {// deep copy from previous channel
#if 0
            this->copyValues(blindData, mValueCount);
            //std::memcpy(blindData, blindData-mValueCount, mValueCount*sizeof(SrcValueT));
#else
            nanovdb::forEach(0,mValueCount,1024,[&](const nanovdb::Range1D &r){
                SrcValueT *dst=blindData+r.begin(), *end=dst+r.size(), *src=dst-mValueCount;
                while(dst!=end) *dst++ = *src++;
            });
#endif
        } else {
            this->copyValues(blindData, mValueCount);
        }
    }
}// IndexGridBuilder::processChannels

} // namespace nanovdb

#endif // NANOVDB_INDEXGRIDBUILDER_H_HAS_BEEN_INCLUDED
