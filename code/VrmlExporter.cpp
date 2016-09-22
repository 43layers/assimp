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



#if !defined(ASSIMP_BUILD_NO_EXPORT) && !defined(ASSIMP_BUILD_NO_VRML_EXPORTER)

#include "VrmlExporter.h"
#include <assimp/version.h>
#include <assimp/IOSystem.hpp>
#include <assimp/scene.h>
#include <assimp/Exporter.hpp>
#include <memory>
#include "Exceptional.h"
#include "ByteSwapper.h"

using namespace Assimp;
namespace Assimp {

void ExportSceneVrml(const char* pFile, IOSystem* pIOSystem, const aiScene* pScene, const ExportProperties* pProperties)
{
  VrmlExporter exporter(pFile, pScene);
  std::unique_ptr<IOStream> outfile(pIOSystem->Open(pFile, "wt"));
  if (outfile == NULL) {
    throw DeadlyExportError("Could not open output .wrl file: " + std::string(pFile));
  }
  outfile->Write(exporter.mOutput.str().c_str(), static_cast<size_t>(exporter.mOutput.tellp()), 1);
}

} // namespace Assimp

VrmlExporter :: VrmlExporter(const char* _filename, const aiScene* pScene)
: filename(_filename)
, endl("\n")
{
    const std::locale& l = std::locale("C");
    mOutput.imbue(l);
    mOutput.precision(16);
    const std::string& name = "AssimpScene";

    mOutput << "#VRML V2.0 utf8" << endl;
    mOutput << "# File produced by Open Asset Import Library (http://www.assimp.sf.net)" << endl;
    mOutput << "# (assimp v" << aiGetVersionMajor() << '.' << aiGetVersionMinor() << '.' << aiGetVersionRevision() << ")" << endl  << endl;


    
}

#endif
