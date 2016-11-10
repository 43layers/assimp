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



#ifndef ASSIMP_BUILD_NO_EXPORT
#ifndef ASSIMP_BUILD_NO_VRML_EXPORTER

#include <iostream>

#include "VrmlExporter.h"
#include "Exceptional.h"
#include "StringComparison.h"
#include <assimp/version.h>
#include <assimp/IOSystem.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/material.h>
#include <assimp/scene.h>
#include <memory>

#define INDENT_WIDTH 2
#define Indent(x) std::string(INDENT_WIDTH * x, ' ')
#define PrintLine(x) mOutput << Indent(mIndent) << x << endl
#define PrintLinePush(x) PrintLine(x); PushIndent()
#define PrintLinePop(x) PopIndent(); PrintLine(x)

using namespace Assimp;
namespace Assimp    {

// ------------------------------------------------------------------------------------------------
// Worker function for exporting a scene to Wavefront OBJ. Prototyped and registered in Exporter.cpp
void ExportSceneVrml(const char* pFile,IOSystem* pIOSystem, const aiScene* pScene, const ExportProperties* pProperties)
{
    // invoke the exporter
    VrmlExporterV2 exporter(pFile, pScene);

    // we're still here - export successfully completed. Write both the main OBJ file and the material script
    std::unique_ptr<IOStream> outfile (pIOSystem->Open(pFile,"wt"));
    if(outfile == NULL) {
        throw DeadlyExportError("could not open output .obj file: " + std::string(pFile));
    }
    outfile->Write( exporter.mOutput.str().c_str(), static_cast<size_t>(exporter.mOutput.tellp()),1);
}

} // namespace Assimp

static const std::string MaterialExt = ".mtl";

// ------------------------------------------------------------------------------------------------
VrmlExporterV2::VrmlExporterV2(const char* _filename, const aiScene* pScene)
: filename(_filename)
, pScene(pScene)
, endl("\n")
{
    // make sure that all formatting happens using the standard, C locale and not the user's current locale
    const std::locale& l = std::locale("C");
    mOutput.imbue(l);
    mOutput.precision(16);

    WriteFile();
}

void VrmlExporterV2::PushIndent() {
  mIndent += 1;
}

void VrmlExporterV2::PopIndent() {
  if (mIndent == 0) return;
  mIndent -= 1;
}


// ------------------------------------------------------------------------------------------------
void VrmlExporterV2::WriteHeader(std::ostringstream& out)
{
    PrintLine("#VRML V2.0 utf8");
    PrintLine("# File produced by Open Asset Import Library (http://www.assimp.sf.net)");
    PrintLine("# (assimp v" << aiGetVersionMajor() << '.' << aiGetVersionMinor() << '.' << aiGetVersionRevision() << ")" << endl);
    mOutput << "NavigationInfo {" << endl << "  type [ \"EXAMINE\", \"ANY\"]" << endl << '}' << endl;
}

// ------------------------------------------------------------------------------------------------
void VrmlExporterV2::WriteFile()
{
    WriteHeader(mOutput);

    // collect mesh geometry
    aiMatrix4x4 mBase;
    AddNode(pScene->mRootNode, mBase);

    // now write all mesh instances
    for(const MeshInstance& m : meshes) {
        PrintLinePush("Transform {");
        PrintLinePush("children [");
        PrintLinePush("Shape {");

        // write appearance
        if (!m.textureFilename.empty()) {
            PrintLinePush("appearance Appearance {");
            PrintLine("texture ImageTexture { url \"" << m.textureFilename << "\" }");
            PrintLinePop("} # appearance");
        }

        PrintLinePush("geometry IndexedFaceSet {");
        PrintLine("solid TRUE");
        // write geometry
        {

          {
            PrintLinePush("coord Coordinate {");
            PrintLinePush("point [");
            m.vpMap.getVectors(positionsVec);
            for (const aiVector3D& v : positionsVec) {
              PrintLine(v.x << " " << v.y << " " << v.z << ',');
            }

            PrintLinePop("] # point");
            PrintLinePop("} # coord Coordinate");
          }
          {
            PrintLinePush("coordIndex [");
            for (const Face& f : m.faces) {
              mOutput << Indent(mIndent);
              for (const FaceVertex& fv : f.indices) {
                mOutput << fv.vp-1 << ',' ;
              }
              mOutput << "-1," << endl;
            }
            PrintLinePop("] # coordIndex");
          }
        }
        // write texture coordinates
        m.vtMap.getVectors(texCoordsVec);
        if (texCoordsVec.size() > 0) {
            PrintLinePush("texCoord TextureCoordinate {");
            PrintLinePush("point [");
            for (const aiVector3D& v : texCoordsVec) {
                PrintLine(v.x << " " << v.y << ",");
            }
            PrintLinePop("] # point");
            PrintLinePop("} # texCoord TextureCoordinate");

            PrintLinePush("texCoordIndex [");
            for (const Face& f : m.faces) {
              mOutput << Indent(mIndent);
              for (const FaceVertex& fv : f.indices) {
                mOutput << fv.vt-1 << ',';
              }
              mOutput << "-1," << endl;
            }
            PrintLinePop("] # texCoordIndex");
        }
        PrintLinePop("} # geometry IndexedFaceSet");

        PrintLinePop("} # Shape");
        PrintLinePop("] # children");
        PrintLinePop("} # Transform");
    }
}

// ------------------------------------------------------------------------------------------------
int VrmlExporterV2::vecIndexMap::getIndex(const aiVector3D& vec)
{
    vecIndexMap::dataType::iterator vertIt = vecMap.find(vec);
    // vertex already exists, so reference it
    if(vertIt != vecMap.end()){
        return vertIt->second;
    }
    vecMap[vec] = mNextIndex;
    int ret = mNextIndex;
    mNextIndex++;
    return ret;
}

// ------------------------------------------------------------------------------------------------
void VrmlExporterV2::vecIndexMap::getVectors( std::vector<aiVector3D>& vecs ) const
{
    vecs.resize(vecMap.size());
    for(vecIndexMap::dataType::const_iterator it = vecMap.begin(); it != vecMap.end(); ++it){
        vecs[it->second-1] = it->first;
    }
}

// ------------------------------------------------------------------------------------------------
void VrmlExporterV2::AddMesh(const aiString& name, const aiMesh* m, const aiMatrix4x4& mat)
{
    meshes.push_back(MeshInstance());
    MeshInstance& mesh = meshes.back();

    unsigned int materialIndex = m->mMaterialIndex;
    const aiMaterial* pMaterial = pScene->mMaterials[materialIndex];
    aiString s;
    std::string diffuseTextureFilename;
    if (AI_SUCCESS == pMaterial->Get(AI_MATKEY_TEXTURE_DIFFUSE(0), s)) {
      mesh.textureFilename = s.data;
    }

    mesh.name = std::string(name.data,name.length) + (m->mName.length ? "_" + std::string(m->mName.data,m->mName.length) : "");

    mesh.faces.resize(m->mNumFaces);

    for(unsigned int i = 0; i < m->mNumFaces; ++i) {
        const aiFace& f = m->mFaces[i];

        Face& face = mesh.faces[i];
        if (f.mNumIndices != 3) continue;
        switch (f.mNumIndices) {
            case 1:
                face.kind = 'p';
                continue;
            case 2:
                face.kind = 'l';
                break;
            default:
                face.kind = 'f';
        }
        face.indices.resize(f.mNumIndices);

        for(unsigned int a = 0; a < f.mNumIndices; ++a) {
            const unsigned int idx = f.mIndices[a];

            aiVector3D vert = mat * m->mVertices[idx];
            face.indices[a].vp = mesh.vpMap.getIndex(vert);

            //TODO: normals support
            face.indices[a].vn = 0;

            if (m->mTextureCoords[0]) {
                face.indices[a].vt = mesh.vtMap.getIndex(m->mTextureCoords[0][idx]);
            }
            else{
                face.indices[a].vt = 0;
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------
void VrmlExporterV2::AddNode(const aiNode* nd, const aiMatrix4x4& mParent)
{
    const aiMatrix4x4& mAbs = mParent * nd->mTransformation;

    for(unsigned int i = 0; i < nd->mNumMeshes; ++i) {
        AddMesh(nd->mName, pScene->mMeshes[nd->mMeshes[i]], mAbs);
    }

    for(unsigned int i = 0; i < nd->mNumChildren; ++i) {
        AddNode(nd->mChildren[i], mAbs);
    }
}

// ------------------------------------------------------------------------------------------------

#endif // ASSIMP_BUILD_NO_OBJ_EXPORTER
#endif // ASSIMP_BUILD_NO_EXPORT
