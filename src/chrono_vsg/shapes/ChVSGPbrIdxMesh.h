#ifndef CH_VSG_PBR_IDX_MESH_H
#define CH_VSG_PBR_IDX_MESH_H

#include <iostream>
#include <string>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChAsset.h"
#include "chrono_vsg/core/ChApiVSG.h"
#include "chrono_vsg/resources/ChVSGSettings.h"
#include "chrono_vsg/resources/ChVSGPbrMaterial.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

class CH_VSG_API ChVSGPbrIdxMesh {
  public:
    ChVSGPbrIdxMesh(std::shared_ptr<ChBody> body,
                     std::shared_ptr<ChAsset> asset,
                     vsg::ref_ptr<vsg::MatrixTransform> transform);

    virtual void Initialize(vsg::vec3& lightPosition, ChVSGPbrMaterial& mat, std::string& texFilePath) = 0;
    vsg::ref_ptr<vsg::Node> createVSGNode(DrawMode drawMode);
 
  protected:
    vsg::ref_ptr<vsg::ShaderStage> readVertexShader(std::string filePath);
    vsg::ref_ptr<vsg::ShaderStage> readFragmentShader(std::string filePath);
    vsg::ref_ptr<vsg::vec4Array2D> createRGBATexture(std::string filePath);

    vsg::vec3 m_lightPosition;
    std::string m_textureFilePath;

    vsg::ref_ptr<vsg::vec3Array> m_vertices;
    vsg::ref_ptr<vsg::vec3Array> m_normals;
    vsg::ref_ptr<vsg::vec2Array> m_texcoords;
    vsg::ref_ptr<vsg::ushortArray> m_indices;

    vsg::vec3 m_albedo;
    float m_metallic;
    float m_roughness;
    float m_ao;

    std::shared_ptr<ChBody> m_bodyPtr;
    std::shared_ptr<ChAsset> m_assetPtr;
    vsg::ref_ptr<vsg::MatrixTransform> m_transform;
};
}  // namespace vsg3d
}  // namespace chrono
#endif