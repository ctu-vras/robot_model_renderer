/*
This file is part of MeshMagick - An Ogre mesh file manipulation tool.
Copyright (C) 2007-2010 Steve Streeting

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// Taken from https://github.com/OGRECave/meshmagick/blob/master/include/MmOptimiseTool.h and slightly edited
// Added recomputeNormals() function.

#pragma once

#include <OgreMesh.h>
#include <OgreMeshSerializer.h>
#include <OgreSubMesh.h>

#include <robot_model_renderer/ogre_helpers/ogre_vector.h>

namespace robot_model_renderer
{
  struct UniqueVertex
  {
    Ogre::Vector3 position;
    Ogre::Vector3 normal;
    Ogre::Vector4 tangent;
    Ogre::Vector3 binormal;
    Ogre::Vector3 uv[OGRE_MAX_TEXTURE_COORD_SETS];
    unsigned short num_uv_sets;

    UniqueVertex();

    UniqueVertex(const Ogre::VertexDeclaration::VertexElementList& elems, const std::vector<uint8_t*>& buffers);

  };

  class MeshOptimizer
  {
  public:
    MeshOptimizer();

    Ogre::MeshPtr optimizeMesh(const Ogre::MeshPtr& mesh,
      float posTolerance = 1e-6, float normTolerance = 1e-6, float uvTolerance = 1e-6);

    struct IndexInfo
    {
      Ogre::uint32 targetIndex;
      bool isOriginal; // was this index the one that created the vertex in the first place

      IndexInfo(Ogre::uint32 targIdx, bool orig) : targetIndex(targIdx), isOriginal(orig) {}
      IndexInfo() {}
    };
    /** Mapping from original vertex index to new (potentially shared) vertex index */
    typedef std::vector<IndexInfo> IndexRemap;
    IndexRemap mIndexRemap;
    std::map<int, IndexRemap> mIndexRemaps;

  protected:
    float mPosTolerance, mNormTolerance, mUVTolerance;

    struct UniqueVertexLess
    {
      float pos_tolerance, norm_tolerance, uv_tolerance;
      unsigned short uvSets;
      bool operator()(const UniqueVertex& a, const UniqueVertex& b) const;

      bool equals(const Ogre::Vector3& a, const Ogre::Vector3& b, Ogre::Real tolerance) const;
      bool equals(const Ogre::Vector4& a, const Ogre::Vector4& b, Ogre::Real tolerance) const;
      bool less(const Ogre::Vector3& a, const Ogre::Vector3& b, Ogre::Real tolerance) const;
      bool less(const Ogre::Vector4& a, const Ogre::Vector4& b, Ogre::Real tolerance) const;
    };

    struct VertexInfo
    {
      Ogre::uint32 oldIndex;
      Ogre::uint32 newIndex;

      VertexInfo(Ogre::uint32 o, Ogre::uint32 n) : oldIndex(o), newIndex(n) {}
      VertexInfo() {}

    };
    /** Map used to efficiently look up vertices that have the same components.
    The second element is the source vertex info.
    */
    typedef std::map<UniqueVertex, VertexInfo, UniqueVertexLess> UniqueVertexMap;

    UniqueVertexMap mUniqueVertexMap;
    /** Ordered list of unique vertices used to write the final reorganised vertex buffer
    */
    typedef std::vector<VertexInfo> UniqueVertexList;
    UniqueVertexList mUniqueVertexList;

    Ogre::VertexData* mTargetVertexData;
    struct IndexDataWithOpType
    {
      Ogre::IndexData* indexData;
      Ogre::RenderOperation::OperationType operationType;

      IndexDataWithOpType(Ogre::IndexData* idata, Ogre::RenderOperation::OperationType opType)
        : indexData(idata), operationType(opType) {}
    };
    typedef std::list<IndexDataWithOpType> IndexDataList;
    IndexDataList mIndexDataList;

    void setTargetVertexData(Ogre::VertexData* vd);
    void addIndexData(Ogre::IndexData* id, Ogre::RenderOperation::OperationType operationType);
    void recomputeNormals();
    bool optimiseGeometry();
    bool calculateDuplicateVertices();
    void rebuildVertexBuffers();
    void remapIndexDataList();
    void remapIndexes(Ogre::IndexData* idata);
    void removeDegenerateFaces();
    void removeDegenerateFaces(Ogre::IndexData* idata);
    Ogre::Mesh::VertexBoneAssignmentList getAdjustedBoneAssignments(
      Ogre::SubMesh::VertexBoneAssignmentList::const_iterator bit,
      Ogre::SubMesh::VertexBoneAssignmentList::const_iterator eit);
    void fixLOD(const Ogre::SubMesh::LODFaceList& lodFaces);
  };
}