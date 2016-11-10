/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2016, assimp team
All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/

/** @file VrmlExporterV2.h
 * Declares the exporter class to write a scene to a Vrml2.0 file
 */
#ifndef AI_VRMLEXPORTERV2_H_INC
#define AI_VRMLEXPORTERV2_H_INC

#include <assimp/types.h>
#include <sstream>
#include <vector>
#include <map>

struct aiScene;
struct aiNode;
struct aiMesh;

namespace Assimp
{

// ------------------------------------------------------------------------------------------------
/** Helper class to export a given scene to an VRML2.0 file. */
// ------------------------------------------------------------------------------------------------
class VrmlExporterV2
{
public:
    /// Constructor for a specific scene to export
    VrmlExporterV2(const char* filename, const aiScene* pScene);

public:

    /// public stringstreams to write all output into
    std::ostringstream mOutput;

private:

    // intermediate data structures
    struct FaceVertex
    {
        FaceVertex()
            : vp(), vn(), vt()
        {
        }

        // 0-indexed
        unsigned int vp,vn,vt;
    };

    struct Face {
        char kind;
        std::vector<FaceVertex> indices;
    };

    struct aiVectorCompare
    {
        bool operator() (const aiVector3D& a, const aiVector3D& b) const
        {
            if(a.x < b.x) return true;
            if(a.x > b.x) return false;
            if(a.y < b.y) return true;
            if(a.y > b.y) return false;
            if(a.z < b.z) return true;
            return false;
        }
    };

    class vecIndexMap
    {
        int mNextIndex;
        typedef std::map<aiVector3D, int, aiVectorCompare> dataType;
        dataType vecMap;
    public:

        vecIndexMap():mNextIndex(1)
        {}

        int getIndex(const aiVector3D& vec);
        void getVectors( std::vector<aiVector3D>& vecs ) const;
    };

    struct MeshInstance {
        std::string name, textureFilename;
        vecIndexMap vpMap, vtMap;
        std::vector<Face> faces;
    };

    void WriteHeader(std::ostringstream& out);
    void WriteFile();

    void AddMesh(const aiString& name, const aiMesh* m, const aiMatrix4x4& mat);
    void AddNode(const aiNode* nd, const aiMatrix4x4& mParent);

private:

    void PushIndent();
    void PopIndent();

    const std::string filename;
    const aiScene* const pScene;

    std::vector<aiVector3D> positionsVec, texCoordsVec;

    unsigned int mIndent = 0;

    std::vector<MeshInstance> meshes;

    // this endl() doesn't flush() the stream
    const std::string endl;
};

}

#endif // AI_VRMLEXPORTERV2_H_INC
