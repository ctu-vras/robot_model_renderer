// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2007-2010 Steve Streeting

// Taken from https://github.com/OGRECave/meshmagick/blob/master/src/MmOptimiseTool.cpp and slightly edited
// Added recomputeNormals() function.
// Removed stuff only relevant for usage inside the meshmagic tool.

#include <robot_model_renderer/robot/mesh_optimizer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <OgreHardwareBuffer.h>
#include <OgreHardwareIndexBuffer.h>
#include <OgreHardwareBufferManager.h>
#include <OgreString.h>
#include <OgreStringConverter.h>

#include <algorithm>
#include <functional>

namespace robot_model_renderer
{

UniqueVertex::UniqueVertex() : position(Ogre::Vector3::ZERO), normal(Ogre::Vector3::ZERO), tangent(Ogre::Vector4::ZERO),
  binormal(Ogre::Vector3::ZERO), num_uv_sets(0u)
{
  std::fill(&uv[0], &uv[OGRE_MAX_TEXTURE_COORD_SETS - 1], Ogre::Vector3::ZERO);
}

UniqueVertex::UniqueVertex(
  const Ogre::VertexDeclaration::VertexElementList& elems, const std::vector<uint8_t*>& buffers) : UniqueVertex()
{
  for (const auto& elem : elems)
  {
    // all float pointers for the moment
    float *pFloat;
    elem.baseVertexPointerToElement(buffers[elem.getSource()], &pFloat);

    switch (elem.getSemantic())
    {
      case Ogre::VES_POSITION:
        this->position.x = *pFloat++;
        this->position.y = *pFloat++;
        this->position.z = *pFloat++;
        break;
      case Ogre::VES_NORMAL:
        this->normal.x = *pFloat++;
        this->normal.y = *pFloat++;
        this->normal.z = *pFloat++;
        break;
      case Ogre::VES_TANGENT:
        this->tangent.x = *pFloat++;
        this->tangent.y = *pFloat++;
        this->tangent.z = *pFloat++;
        // support w-component on tangent if present
        if (Ogre::VertexElement::getTypeCount(elem.getType()) == 4)
          this->tangent.w = *pFloat++;
        break;
      case Ogre::VES_BINORMAL:
        this->binormal.x = *pFloat++;
        this->binormal.y = *pFloat++;
        this->binormal.z = *pFloat++;
        break;
      case Ogre::VES_TEXTURE_COORDINATES:
        // supports up to 4 dimensions
        for (unsigned short dim = 0;  // NOLINT(runtime/int)
            dim < Ogre::VertexElement::getTypeCount(elem.getType()); ++dim)
          this->uv[elem.getIndex()][dim] = *pFloat++;
        ++num_uv_sets;
        break;
      case Ogre::VES_BLEND_INDICES:
      case Ogre::VES_BLEND_WEIGHTS:
      case Ogre::VES_DIFFUSE:
      case Ogre::VES_SPECULAR:
        // No action needed for these semantics.
        break;
    }
  }
}

MeshOptimizer::MeshOptimizer()
  : mPosTolerance(1e-3), mNormTolerance(1e-3), mUVTolerance(1e-3), mTargetVertexData(nullptr)
{
}

Ogre::MeshPtr MeshOptimizer::optimizeMesh(const Ogre::MeshPtr& mesh,
  const float posTolerance, const float normTolerance, const float uvTolerance)
{
  this->mIndexRemaps.clear();

  this->mPosTolerance = posTolerance;
  this->mNormTolerance = normTolerance;
  this->mUVTolerance = uvTolerance;
  bool rebuildEdgeList = false;

  // Shared geometry
  if (mesh->sharedVertexData)
  {
    setTargetVertexData(mesh->sharedVertexData);

    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)  // NOLINT(runtime/int)
    {
      const auto& sm = mesh->getSubMesh(i);
      if (sm->useSharedVertices)
      {
        addIndexData(sm->indexData, sm->operationType);
      }
    }

    if (optimiseGeometry())
    {
      if (mesh->getSkeletonName() != Ogre::StringUtil::BLANK)
      {
        const auto& bas = mesh->getBoneAssignments();
        auto newList = getAdjustedBoneAssignments(bas.begin(), bas.end());
        mesh->clearBoneAssignments();
        for (const auto& boneAssignment : newList)
          mesh->addBoneAssignment(boneAssignment.second);
      }

      for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)  // NOLINT(runtime/int)
      {
        Ogre::SubMesh* sm = mesh->getSubMesh(i);
        if (mesh->getSkeletonName() != Ogre::StringUtil::BLANK)
        {
          const auto& bas = sm->getBoneAssignments();
          auto newList = getAdjustedBoneAssignments(bas.begin(), bas.end());
          sm->clearBoneAssignments();
          for (const auto& boneAssignment : newList)
            sm->addBoneAssignment(boneAssignment.second);
        }
        if (sm->useSharedVertices)
        {
          fixLOD(sm->mLodFaceList);
        }
      }
      rebuildEdgeList = true;
      mIndexRemaps[-1] = mIndexRemap;
    }
  }

  // Dedicated geometry
  for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)  // NOLINT(runtime/int)
  {
    Ogre::SubMesh* sm = mesh->getSubMesh(i);
    if (!sm->useSharedVertices)
    {
      setTargetVertexData(sm->vertexData);
      addIndexData(sm->indexData, sm->operationType);
      if (optimiseGeometry())
      {
        if (mesh->getSkeletonName() != Ogre::StringUtil::BLANK)
        {
          const auto& bas = sm->getBoneAssignments();
          auto newList = getAdjustedBoneAssignments(bas.begin(), bas.end());
          sm->clearBoneAssignments();
          for (auto& boneAssignment : newList)
            sm->addBoneAssignment(boneAssignment.second);
        }

        fixLOD(sm->mLodFaceList);
        rebuildEdgeList = true;
        mIndexRemaps[i] = mIndexRemap;
      }
    }
  }

  mesh->_dirtyState();
  mesh->_updateCompiledBoneAssignments();
  if (rebuildEdgeList && mesh->isEdgeListBuilt())
  {
    // force rebuild of edge list
    mesh->freeEdgeList();
    mesh->buildEdgeList();
  }

  return mesh;
}

void MeshOptimizer::fixLOD(const Ogre::SubMesh::LODFaceList& lodFaces)
{
  for (const auto& l : lodFaces)
  {
    remapIndexes(l);
  }
}

Ogre::Mesh::VertexBoneAssignmentList MeshOptimizer::getAdjustedBoneAssignments(
  Ogre::SubMesh::VertexBoneAssignmentList::const_iterator bit,
  Ogre::SubMesh::VertexBoneAssignmentList::const_iterator eit)
{
  Ogre::Mesh::VertexBoneAssignmentList newList;
  for (; bit != eit; ++bit)
  {
    Ogre::VertexBoneAssignment ass = bit->second;
    IndexInfo& ii = mIndexRemap[ass.vertexIndex];

    // If this is the originating vertex index  we want to add the (adjusted)
    // bone assignments. If it's another vertex that was collapsed onto another
    // then we want to skip the bone assignment since it will just be a duplication.
    if (ii.isOriginal)
    {
      ass.vertexIndex = static_cast<unsigned int>(ii.targetIndex);
      assert(ass.vertexIndex < mUniqueVertexMap.size());
      newList.insert(Ogre::Mesh::VertexBoneAssignmentList::value_type(
        ass.vertexIndex, ass));
    }
  }
  return newList;
}

void MeshOptimizer::setTargetVertexData(Ogre::VertexData* vd)
{
  mTargetVertexData = vd;
  mUniqueVertexMap.clear();
  mUniqueVertexList.clear();
  mIndexDataList.clear();
  mIndexRemap.clear();
}

void MeshOptimizer::addIndexData(Ogre::IndexData* id, const Ogre::RenderOperation::OperationType ot)
{
  mIndexDataList.emplace_back(id, ot);
}

bool MeshOptimizer::optimiseGeometry()
{
  bool verticesChanged = false;
  if (calculateDuplicateVertices())
  {
    rebuildVertexBuffers();
    remapIndexDataList();
    verticesChanged = true;
  }

  removeDegenerateFaces();

  recomputeNormals();

  return verticesChanged;
}

bool MeshOptimizer::calculateDuplicateVertices()
{
  bool duplicates = false;

  // Can't remove duplicates on unindexed geometry, needs to use duplicates
  if (mIndexDataList.empty())
    return false;

  const auto& bindings = mTargetVertexData->vertexBufferBinding->getBindings();

  // Lock all the buffers first
  std::vector<Ogre::HardwareBufferLockGuard<Ogre::HardwareVertexBufferSharedPtr>> buffer_locks;
  std::vector<uint8_t*> buffers;
  buffers.resize(mTargetVertexData->vertexBufferBinding->getLastBoundIndex() + 1);
  for (const auto& [buf_idx, buf] : bindings)
  {
    buffer_locks.emplace_back(buf, Ogre::HardwareBuffer::HBL_READ_ONLY);
    buffers[buf_idx] = static_cast<uint8_t*>(buffer_locks.back().pData);
  }

  for (size_t v = 0; v < mTargetVertexData->vertexCount; ++v)
  {
    UniqueVertex uniqueVertex(mTargetVertexData->vertexDeclaration->getElements(), buffers);

    if (v == 0)
    {
      // set up comparator
      UniqueVertexLess lessObj {mPosTolerance, mNormTolerance, mUVTolerance, uniqueVertex.num_uv_sets};
      mUniqueVertexMap = UniqueVertexMap(lessObj);
    }

    // try to locate equivalent vertex in the list already
    Ogre::uint32 indexUsed;
    bool isOrig = false;
    if (const auto vertexIt = mUniqueVertexMap.find(uniqueVertex); vertexIt != mUniqueVertexMap.end())
    {
      // re-use vertex, remap
      indexUsed = vertexIt->second.newIndex;
      duplicates = true;
    }
    else
    {
      // new vertex
      isOrig = true;
      indexUsed = static_cast<Ogre::uint32>(mUniqueVertexMap.size());
      // store the originating and new vertex index in the unique map
      VertexInfo newInfo(v, indexUsed);
      // lookup
      mUniqueVertexMap[uniqueVertex] = newInfo;
      // ordered
      mUniqueVertexList.push_back(newInfo);
    }
    // Insert remap entry (may map to itself)
    mIndexRemap.emplace_back(indexUsed, isOrig);

    // increment buffer pointers
    for (const auto& [buf_idx, buf] : bindings)
      buffers[buf_idx] += buf->getVertexSize();
  }

  // Were there duplicates?
  return duplicates;
}

void MeshOptimizer::rebuildVertexBuffers()
{
  // We need to build new vertex buffers of the new, reduced size
  auto newBind = Ogre::HardwareBufferManager::getSingleton().createVertexBufferBinding();
  const auto& srcBindings = mTargetVertexData->vertexBufferBinding->getBindings();

  {
    // Lock source and dest buffers and copy data
    typedef Ogre::HardwareBufferLockGuard<Ogre::HardwareVertexBufferSharedPtr> VertexLockGuard;
    std::list<Ogre::HardwareVertexBufferSharedPtr> new_buffers;
    std::list<VertexLockGuard> src_buffer_locks, dest_buffer_locks;
    std::vector<uint8_t*> src_buffers, dest_buffers;
    src_buffers.resize(mTargetVertexData->vertexBufferBinding->getLastBoundIndex()+1);
    dest_buffers.resize(mTargetVertexData->vertexBufferBinding->getLastBoundIndex()+1);
    for (const auto& [buf_idx, buf] : srcBindings)
    {
      src_buffer_locks.emplace_back(buf, Ogre::HardwareBuffer::HBL_READ_ONLY);
      src_buffers[buf_idx] = static_cast<uint8_t*>(src_buffer_locks.back().pData);

      // Add a new vertex buffer and binding
      auto newBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        buf->getVertexSize(), mUniqueVertexList.size(), buf->getUsage(), buf->hasShadowBuffer());
      newBind->setBinding(buf_idx, newBuf);
      new_buffers.push_back(newBuf);

      dest_buffer_locks.emplace_back(new_buffers.back(), Ogre::HardwareBuffer::HBL_DISCARD);
      dest_buffers[buf_idx] = static_cast<uint8_t*>(dest_buffer_locks.back().pData);
    }
    const auto& destBindings = newBind->getBindings();

    // Iterate over the new vertices
    for (const auto& ui : mUniqueVertexList)
    {
      const Ogre::uint32 origVertexIndex = ui.oldIndex;
      // copy vertex from each buffer in turn
      auto srci = srcBindings.begin();
      auto desti = destBindings.begin();
      for (; srci != srcBindings.end(); ++srci, ++desti)
      {
        // determine source pointer
        const auto pSrc = src_buffers[srci->first] + (srci->second->getVertexSize() * origVertexIndex);
        const auto pDest = dest_buffers[desti->first];

        // Copy vertex from source index
        memcpy(pDest, pSrc, desti->second->getVertexSize());

        // // increment destination lock pointer
        dest_buffers[desti->first] += desti->second->getVertexSize();
      }
    }
  }

  // now switch over the bindings, and thus the buffers
  Ogre::VertexBufferBinding* oldBind = mTargetVertexData->vertexBufferBinding;
  mTargetVertexData->vertexBufferBinding = newBind;
  Ogre::HardwareBufferManager::getSingleton().destroyVertexBufferBinding(oldBind);

  // Update vertex count in data
  mTargetVertexData->vertexCount = mUniqueVertexList.size();
}

void MeshOptimizer::remapIndexDataList()
{
  for (const auto& i : mIndexDataList)
    remapIndexes(i.indexData);
}

void MeshOptimizer::remapIndexes(Ogre::IndexData* idata)
{
  // Time to repoint indexes at the new shared vertices
  Ogre::uint16* p16 = nullptr;
  Ogre::uint32* p32 = nullptr;

  // Lock for read & write
  Ogre::HardwareBufferLockGuard index_lock(idata->indexBuffer, Ogre::HardwareBuffer::HBL_NORMAL);
  // Lock for read only, we'll build another list
  if (idata->indexBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
    p32 = static_cast<Ogre::uint32*>(index_lock.pData);
  else
    p16 = static_cast<Ogre::uint16*>(index_lock.pData);

  for (size_t j = 0; j < idata->indexCount; ++j)
  {
    Ogre::uint32 oldIndex = p32? *p32 : *p16;
    Ogre::uint32 newIndex = static_cast<Ogre::uint32>(mIndexRemap[oldIndex].targetIndex);
    assert(newIndex < mUniqueVertexMap.size());
    if (newIndex != oldIndex)
    {
      if (p32)
        *p32 = newIndex;
      else
        *p16 = static_cast<Ogre::uint16>(newIndex);
    }
    if (p32)
      ++p32;
    else
      ++p16;
  }
}

void MeshOptimizer::removeDegenerateFaces()
{
  for (const auto& index : mIndexDataList)
  {
    // Only remove degenerate faces from triangle lists, strips & fans need them
    if (index.operationType == Ogre::RenderOperation::OT_TRIANGLE_LIST)
      removeDegenerateFaces(index.indexData);
  }
}

void MeshOptimizer::removeDegenerateFaces(Ogre::IndexData* idata)
{
  // Remove any faces that do not include 3 unique positions

  // Only for triangle lists
  Ogre::uint16* p16 = nullptr;
  Ogre::uint32* p32 = nullptr;
  Ogre::uint16* pnewbuf16 = nullptr;
  Ogre::uint32* pnewbuf32 = nullptr;
  Ogre::uint16* pdest16 = nullptr;
  Ogre::uint32* pdest32 = nullptr;
  size_t newIndexCount = 0u;

  {
    Ogre::HardwareBufferLockGuard index_lock(idata->indexBuffer, Ogre::HardwareBuffer::HBL_READ_ONLY);
    // Lock for read only, we'll build another list
    if (idata->indexBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
    {
      p32 = static_cast<Ogre::uint32*>(index_lock.pData);
      pnewbuf32 = pdest32 = OGRE_ALLOC_T(Ogre::uint32, idata->indexCount, Ogre::MEMCATEGORY_GENERAL);
    }
    else
    {
      p16 = static_cast<Ogre::uint16*>(index_lock.pData);
      pnewbuf16 = pdest16 = OGRE_ALLOC_T(Ogre::uint16, idata->indexCount, Ogre::MEMCATEGORY_GENERAL);
    }

    const auto& posElem = mTargetVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
    const auto posBuf = mTargetVertexData->vertexBufferBinding->getBuffer(posElem->getSource());
    const Ogre::HardwareBufferLockGuard pos_lock(posBuf, Ogre::HardwareBuffer::HBL_READ_ONLY);
    auto pVertBase = static_cast<unsigned char*>(pos_lock.pData);
    const auto vsize = posBuf->getVertexSize();

    for (size_t j = 0; j < idata->indexCount; j += 3)
    {
      Ogre::uint32 i0 = p32 ? *p32++ : *p16++;
      Ogre::uint32 i1 = p32 ? *p32++ : *p16++;
      Ogre::uint32 i2 = p32 ? *p32++ : *p16++;

      float* pPosVert;
      posElem->baseVertexPointerToElement(pVertBase + i0 * vsize, &pPosVert);
      Ogre::Vector3 v0(pPosVert[0], pPosVert[1], pPosVert[2]);
      posElem->baseVertexPointerToElement(pVertBase + i1 * vsize, &pPosVert);
      Ogre::Vector3 v1(pPosVert[0], pPosVert[1], pPosVert[2]);
      posElem->baseVertexPointerToElement(pVertBase + i2 * vsize, &pPosVert);
      Ogre::Vector3 v2(pPosVert[0], pPosVert[1], pPosVert[2]);

      // No double-indexing
      bool validTri = i0 != i1 && i1 != i2 && i0 != i2;
      // no equal positions
      validTri = validTri &&
        !v0.positionEquals(v1, mPosTolerance) &&
        !v1.positionEquals(v2, mPosTolerance) &&
        !v0.positionEquals(v2, mPosTolerance);
      if (validTri)
      {
        // Make sure triangle has some area
        Ogre::Vector3 vec1 = v1 - v0;
        Ogre::Vector3 vec2 = v2 - v0;
        // triangle area is 1/2 magnitude of the cross-product of 2 sides
        // if zero, not a valid triangle
        validTri = !Ogre::Math::RealEqual(static_cast<Ogre::Real>(0.0f),
          static_cast<Ogre::Real>(0.5f * vec1.crossProduct(vec2).length()), 1e-08f);
      }

      if (validTri)
      {
        if (pdest32)
        {
          *pdest32++ = i0;
          *pdest32++ = i1;
          *pdest32++ = i2;
        }
        else
        {
          *pdest16++ = static_cast<Ogre::uint16>(i0);
          *pdest16++ = static_cast<Ogre::uint16>(i1);
          *pdest16++ = static_cast<Ogre::uint16>(i2);
        }
        newIndexCount += 3;
      }
    }
  }

  if (newIndexCount != idata->indexCount)
  {
    // Did we remove all the faces? (really bad data only, but I've seen it happen)
    if (newIndexCount > 0)
    {
      // we eliminated one or more faces
      auto newIBuf = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        idata->indexBuffer->getType(), newIndexCount, idata->indexBuffer->getUsage());
      if (pdest32)
        newIBuf->writeData(0, sizeof(Ogre::uint32) * newIndexCount, pnewbuf32, true);
      else
        newIBuf->writeData(0, sizeof(Ogre::uint16) * newIndexCount, pnewbuf16, true);
      idata->indexBuffer = newIBuf;
    }
    else
    {
      idata->indexBuffer.setNull();
    }
    idata->indexCount = newIndexCount;
  }

  OGRE_FREE(pnewbuf16, Ogre::MEMCATEGORY_GENERAL);
  OGRE_FREE(pnewbuf32, Ogre::MEMCATEGORY_GENERAL);
}

inline void get_triangle_vertices(const Ogre::RenderOperation::OperationType op,
  const Ogre::uint16* p16, const Ogre::uint32* p32, const size_t j,
  Ogre::uint32& i0, Ogre::uint32& i1, Ogre::uint32& i2)
{
  if (op == Ogre::RenderOperation::OT_TRIANGLE_LIST)
  {
    i0 = p32 ? p32[j + 0] : p16[j + 0];
    i1 = p32 ? p32[j + 1] : p16[j + 1];
    i2 = p32 ? p32[j + 2] : p16[j + 2];
  }
  else if (op == Ogre::RenderOperation::OT_TRIANGLE_STRIP)
  {
    if (j % 2 == 0)
    {
      i0 = p32 ? p32[j + 0] : p16[j + 0];
      i1 = p32 ? p32[j + 1] : p16[j + 1];
      i2 = p32 ? p32[j + 2] : p16[j + 2];
    }
    else
    {
      i2 = p32 ? p32[j + 0] : p16[j + 0];
      i1 = p32 ? p32[j + 1] : p16[j + 1];
      i0 = p32 ? p32[j + 2] : p16[j + 2];
    }
  }
  else
  {
    i0 = p32 ? p32[0] : p16[0];
    i1 = p32 ? p32[j + 1] : p16[j + 1];
    i2 = p32 ? p32[j + 2] : p16[j + 2];
  }
}

inline float angleBetweenVectors(const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2)
{
  Eigen::AngleAxisf a(Eigen::Quaternionf::FromTwoVectors(vec1, vec2));
  return a.angle();
}

void MeshOptimizer::recomputeNormals()
{
  const auto& pos_elem = mTargetVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
  const auto& normals_elem = mTargetVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);

  // If the mesh has no positions or normals, there is nothing to do
  if (pos_elem == nullptr || normals_elem == nullptr)
    return;

  const auto& pos_buffer = mTargetVertexData->vertexBufferBinding->getBuffer(pos_elem->getSource());
  const auto& norm_buffer = mTargetVertexData->vertexBufferBinding->getBuffer(normals_elem->getSource());

  const size_t pos_vertex_size = pos_buffer->getVertexSize();
  const size_t norm_vertex_size = norm_buffer->getVertexSize();

  // The Eigen::Map<> expressions are a kind of Eigen-casted reference to the data, so assigning to the Map
  // also changes the underlying raw data buffer.

  // Zero out vertex normals
  {
    Ogre::HardwareBufferLockGuard norm_lock(norm_buffer, Ogre::HardwareBuffer::LockOptions::HBL_WRITE_ONLY);
    const auto norm_vertices_data = static_cast<uint8_t*>(norm_lock.pData);

    float* normal {nullptr};
    for (size_t v = 0u; v < mTargetVertexData->vertexCount; ++v)
    {
      normals_elem->baseVertexPointerToElement(norm_vertices_data + norm_vertex_size * v, &normal);
      Eigen::Map<Eigen::Vector3f>(normal).setZero();
    }
  }

  // Add triangle normals to all neighbor vertices (no normalization yet)
  for (const auto& index : mIndexDataList)
  {
    const auto op = index.operationType;

    if (op != Ogre::RenderOperation::OT_TRIANGLE_LIST && op != Ogre::RenderOperation::OT_TRIANGLE_STRIP &&
      op != Ogre::RenderOperation::OT_TRIANGLE_FAN)
      continue;

    const auto idata = index.indexData;
    if (idata->indexCount < 3)
      continue;

    Ogre::uint16* p16 = nullptr;
    Ogre::uint32* p32 = nullptr;

    const Ogre::HardwareBufferLockGuard index_lock(idata->indexBuffer, Ogre::HardwareBuffer::HBL_READ_ONLY);
    if (idata->indexBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
      p32 = static_cast<Ogre::uint32*>(index_lock.pData);
    else
      p16 = static_cast<Ogre::uint16*>(index_lock.pData);

    typedef Ogre::HardwareBufferLockGuard<Ogre::HardwareVertexBufferSharedPtr> VertexLockGuard;
    const VertexLockGuard pos_lock(pos_buffer, Ogre::HardwareBuffer::LockOptions::HBL_READ_ONLY);
    // We can't right-away lock norm_buffer because it can be the same buffer as pos_buffer
    // only lock it if it's different
    std::unique_ptr<VertexLockGuard> norm_lock;
    if (pos_buffer != norm_buffer)
      norm_lock = std::make_unique<VertexLockGuard>(norm_buffer, Ogre::HardwareBuffer::LockOptions::HBL_NORMAL);

    const auto pos_vertices_data = static_cast<unsigned char*>(pos_lock.pData);
    const auto norm_vertices_data = static_cast<unsigned char*>(norm_lock ? norm_lock->pData : pos_lock.pData);

    const auto max_idx = op == Ogre::RenderOperation::OT_TRIANGLE_LIST ? idata->indexCount : idata->indexCount - 2;
    const auto inc_idx = op == Ogre::RenderOperation::OT_TRIANGLE_LIST ? 3 : 1;

    std::vector<Ogre::Vector3> triangleNormals;
    triangleNormals.reserve(max_idx);

    for (size_t j = 0; j < max_idx; j += inc_idx)
    {
      Ogre::uint32 i0, i1, i2;
      get_triangle_vertices(op, p16, p32, j, i0, i1, i2);

      // No double-indexing
      const auto valid_indices = i0 != i1 && i1 != i2 && i0 != i2;
      if (!valid_indices)
        continue;

      float* pos_vertex;
      pos_elem->baseVertexPointerToElement(pos_vertices_data + (i0 * pos_vertex_size), &pos_vertex);
      const Eigen::Map<Eigen::Vector3f> p0(pos_vertex);
      pos_elem->baseVertexPointerToElement(pos_vertices_data + (i1 * pos_vertex_size), &pos_vertex);
      const Eigen::Map<Eigen::Vector3f> p1(pos_vertex);
      pos_elem->baseVertexPointerToElement(pos_vertices_data + (i2 * pos_vertex_size), &pos_vertex);
      const Eigen::Map<Eigen::Vector3f> p2(pos_vertex);

      const Eigen::Vector3f vec0 = p0 - p1;
      const Eigen::Vector3f vec1 = p1 - p2;
      const Eigen::Vector3f vec2 = p2 - p0;

      // Skip triangles with too short edges
      if (std::min({vec0.norm(), vec1.norm(), vec2.norm()}) < 1e-6f)
        continue;

      auto tri_normal = vec0.cross(vec1);
      // Skip degenerate triangles
      if (tri_normal.norm() < 1e-6f)
        continue;
      tri_normal.normalize();

      // Calculate angle between the two vectors
      const auto ang0 = angleBetweenVectors(-vec0, vec2);
      const auto ang1 = angleBetweenVectors(vec0, -vec1);
      const auto ang2 = angleBetweenVectors(-vec2, vec1);

      float* norm_vertex;
      normals_elem->baseVertexPointerToElement(norm_vertices_data + i0 * norm_vertex_size, &norm_vertex);
      Eigen::Map<Eigen::Vector3f>(norm_vertex) += tri_normal * ang0;
      normals_elem->baseVertexPointerToElement(norm_vertices_data + i1 * norm_vertex_size, &norm_vertex);
      Eigen::Map<Eigen::Vector3f>(norm_vertex) += tri_normal * ang1;
      normals_elem->baseVertexPointerToElement(norm_vertices_data + i2 * norm_vertex_size, &norm_vertex);
      Eigen::Map<Eigen::Vector3f>(norm_vertex) += tri_normal * ang2;
    }
  }

  // Normalize the vertex normals
  {
    const Ogre::HardwareBufferLockGuard norm_lock(norm_buffer, Ogre::HardwareBuffer::LockOptions::HBL_NORMAL);
    const auto norm_vertices_data = static_cast<unsigned char*>(norm_lock.pData);

    float* normal;
    for (size_t v = 0u; v < mTargetVertexData->vertexCount; ++v)
    {
      normals_elem->baseVertexPointerToElement(norm_vertices_data + v * norm_vertex_size, &normal);
      Eigen::Map<Eigen::Vector3f>(normal).normalize();
    }
  }
}

bool MeshOptimizer::UniqueVertexLess::equals(
  const Ogre::Vector3& a, const Ogre::Vector3& b, Ogre::Real tolerance) const
{
  // note during this comparison we treat directions as positions
  // becuase we're interested in numerical equality, not semantics
  // and some of these might be null and thus not be a valid direction
  return a.positionEquals(b, tolerance);
}

bool MeshOptimizer::UniqueVertexLess::equals(
  const Ogre::Vector4& a, const Ogre::Vector4& b, Ogre::Real tolerance) const
{
  // no built-in position equals
  for (int i = 0; i < 4; ++i)
  {
    if (Ogre::Math::RealEqual(a[i], b[i], tolerance))
      return true;
  }
  return false;
}

bool MeshOptimizer::UniqueVertexLess::less(
  const Ogre::Vector3& a, const Ogre::Vector3& b, Ogre::Real tolerance) const
{
  // don't use built-in operator, we need sorting
  for (int i = 0; i < 3; ++i)
  {
    if (!Ogre::Math::RealEqual(a[i], b[i], tolerance))
      return a[i] < b[i];
  }
  // should never get here if equals() has been checked first
  return a.x < b.x;
}

bool MeshOptimizer::UniqueVertexLess::less(
  const Ogre::Vector4& a, const Ogre::Vector4& b, Ogre::Real tolerance) const
{
  // don't use built-in operator, we need sorting
  for (int i = 0; i < 4; ++i)
  {
    if (!Ogre::Math::RealEqual(a[i], b[i], tolerance))
      return a[i] < b[i];
  }
  // should never get here if equals() has been checked first
  return a.x < b.x;
}

bool MeshOptimizer::UniqueVertexLess::operator ()(
  const UniqueVertex &a,
  const UniqueVertex &b) const
{
  if (!equals(a.position, b.position, pos_tolerance))
  {
    return less(a.position, b.position, pos_tolerance);
  }
  else if (!equals(a.normal, b.normal, norm_tolerance))
  {
    return less(a.normal, b.normal, norm_tolerance);
  }
  else if (!equals(a.tangent, b.tangent, norm_tolerance))
  {
    return less(a.tangent, b.tangent, norm_tolerance);
  }
  else if (!equals(a.binormal, b.binormal, norm_tolerance))
  {
    return less(a.binormal, b.binormal, norm_tolerance);
  }
  else
  {
    // position, normal, tangent and binormal are all the same, try UVs
    for (unsigned short i = 0; i < uvSets; ++i)  // NOLINT(runtime/int)
    {
      if (!equals(a.uv[i], b.uv[i], uv_tolerance))
      {
        return less(a.uv[i], b.uv[i], uv_tolerance);
      }
    }
    // if we get here, must be equal (with tolerance)
    return false;
  }
}

}
