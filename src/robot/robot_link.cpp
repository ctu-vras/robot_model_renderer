// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: 2008 Willow Garage, Inc.
// SPDX-FileCopyrightText: Czech Technical University in Prague

// This file is compiled from rviz and slightly edited to be usable in this package.
// Added inflateMesh() method (Martin Pecka, Czech Technical University in Prague)

#include <robot_model_renderer/robot/robot_link.h>

#include <map>
#include <string>

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreRibbonTrail.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreSubEntity.h>
#include <OgreSubMesh.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include <resource_retriever/retriever.h>
#include <ros/console.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>

#include <robot_model_renderer/ogre_helpers/object.h>
#include <robot_model_renderer/ogre_helpers/shape.h>
#include <robot_model_renderer/robot/mesh_loader.h>
#include <robot_model_renderer/robot/mesh_optimizer.h>
#include <robot_model_renderer/robot/robot.h>
#include <robot_model_renderer/robot/robot_joint.h>
#include <robot_model_renderer/robot/shape_filter.h>
#include <robot_model_renderer/robot/shape_inflation_registry.h>

namespace fs = boost::filesystem;

namespace robot_model_renderer
{

RobotLink::RobotLink(Robot* robot, const urdf::LinkConstSharedPtr& link, const std::string& parent_joint_name,
  const std::shared_ptr<ShapeFilter>& shape_filter,
  const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry)
  : robot_(robot), scene_manager_(robot->getSceneManager()), name_(link->name), parent_joint_name_(parent_joint_name),
    visual_node_(nullptr), collision_node_(nullptr), robot_alpha_(1.0), only_render_depth_(false),
    material_mode_flags_(ORIGINAL), enabled_(true)
{
  visual_node_ = robot_->getVisualNode()->createChildSceneNode();
  collision_node_ = robot_->getCollisionNode()->createChildSceneNode();

  {
    // create material for coloring links
    std::string material_name = "robot link " + link->name + ":color material";
    color_material_ = Ogre::MaterialPtr(new Ogre::Material(
        nullptr, material_name, 0, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME));
    color_material_->setReceiveShadows(false);
    color_material_->getTechnique(0)->setLightingEnabled(true);
  }

  {
    // create material for drawing masks
    std::string material_name = "robot link " + link->name + ":mask material";
    mask_material_ = Ogre::MaterialManager::getSingleton().getByName("BaseWhiteNoLighting")->clone(
      material_name, true, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }

  // create the ogre objects to display

  if (shape_filter->isVisualAllowed())
  {
    createVisual(link, shape_filter, shape_inflation_registry);
  }

  if (shape_filter->isCollisionAllowed())
  {
    createCollision(link, shape_filter, shape_inflation_registry);
  }
}

RobotLink::~RobotLink()
{
  for (const auto& visual_mesh : visual_meshes_)
  {
    scene_manager_->destroyEntity(visual_mesh);
  }

  for (const auto& collision_mesh : collision_meshes_)
  {
    scene_manager_->destroyEntity(collision_mesh);
  }

  scene_manager_->destroySceneNode(visual_node_);
  scene_manager_->destroySceneNode(collision_node_);
}

std::string RobotLink::getGeometryErrors() const
{
  return errors_;
}

bool RobotLink::hasGeometry() const
{
  return visual_meshes_.size() + collision_meshes_.size() > 0;
}

bool RobotLink::getEnabled() const
{
  return this->enabled_;
}

void RobotLink::setRobotAlpha(const float a)
{
  robot_alpha_ = a;
  updateAlpha();
}

void RobotLink::setRenderQueueGroup(const Ogre::uint8 group)
{
  auto child_it = visual_node_->getChildIterator();
  while (child_it.hasMoreElements())
  {
    const auto child = dynamic_cast<Ogre::SceneNode*>(child_it.getNext());
    if (child)
    {
      auto object_it = child->getAttachedObjectIterator();
      while (object_it.hasMoreElements())
      {
        Ogre::MovableObject* obj = object_it.getNext();
        obj->setRenderQueueGroup(group);
      }
    }
  }
}

void RobotLink::setOnlyRenderDepth(const bool onlyRenderDepth)
{
  setRenderQueueGroup(onlyRenderDepth ? Ogre::RENDER_QUEUE_BACKGROUND : Ogre::RENDER_QUEUE_MAIN);
  only_render_depth_ = onlyRenderDepth;
  updateAlpha();
}

void RobotLink::updateAlpha()
{
  float link_alpha = 1.0;
  for (auto& [subEntity, material] : materials_)
  {
    auto& [activeMat, originalMat] = material;

    if (only_render_depth_)
    {
      activeMat->setColourWriteEnabled(false);
      activeMat->setDepthWriteEnabled(true);
    }
    else
    {
      Ogre::ColourValue color = activeMat->getTechnique(0)->getPass(0)->getDiffuse();
      const float material_alpha = originalMat->getTechnique(0)->getPass(0)->getDiffuse().a;
      color.a = robot_alpha_ * material_alpha * link_alpha;
      activeMat->setDiffuse(color);

      if (color.a < 0.9998) // transparent
      {
        activeMat->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        activeMat->setDepthWriteEnabled(false);
      }
      else if (activeMat == originalMat)
      {
        activeMat->setSceneBlending(Ogre::SBT_REPLACE);
        activeMat->setDepthWriteEnabled(true);
      }
      else // restore original material
      {
        originalMat->copyDetailsTo(activeMat);
      }
    }
  }

  Ogre::ColourValue color = color_material_->getTechnique(0)->getPass(0)->getDiffuse();
  color.a = robot_alpha_ * link_alpha;
  color_material_->setDiffuse(color);

  if (color.a < 0.9998)
  {
    color_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    color_material_->setDepthWriteEnabled(false);
  }
  else
  {
    color_material_->setSceneBlending(Ogre::SBT_REPLACE);
    color_material_->setDepthWriteEnabled(true);
  }
}

void RobotLink::updateVisibility()
{
  const bool enabled = getEnabled() && robot_->isVisible();

  if (visual_node_)
  {
    visual_node_->setVisible(enabled && robot_->isVisualVisible());
  }
  if (collision_node_)
  {
    collision_node_->setVisible(enabled && robot_->isCollisionVisible());
  }
}

Ogre::MaterialPtr RobotLink::getMaterialForLink(
  const urdf::LinkConstSharedPtr& link, urdf::MaterialConstSharedPtr material)
{
  // only the first visual's material actually comprises color values, all others only have the name
  // hence search for the first visual with given material name (better fix the bug in urdf parser)
  if (material && !material->name.empty())
  {
    for (const auto& visual : link->visual_array)
    {
      if (visual->material_name == material->name)
      {
        material = visual->material;
        break;
      }
    }
  }
  if (!material && link->visual && link->visual->material)
    material = link->visual->material; // fallback to visual's material

  std::string name = "robot link " + link->name;
  if (material)
    name += ":" + material->name;

  auto mat = Ogre::MaterialPtr(
    new Ogre::Material(nullptr, name, 0, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME));

  if (!material)
  {
    // clone default material (for modification by link)
    *mat = *Ogre::MaterialManager::getSingleton().getByName("RVIZ/ShadedRed");
    return mat;
  }

  mat->getTechnique(0)->setLightingEnabled(true);
  if (material->texture_filename.empty())
  {
    const urdf::Color& col = material->color;
    mat->getTechnique(0)->setAmbient(col.r * 0.5, col.g * 0.5, col.b * 0.5);
    mat->getTechnique(0)->setDiffuse(col.r, col.g, col.b, col.a);
  }
  else
  {
    std::string filename = material->texture_filename;
    if (!Ogre::TextureManager::getSingleton().resourceExists(filename))
    {
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever.get(filename);
      }
      catch (resource_retriever::Exception& e)
      {
        ROS_ERROR("%s", e.what());
      }

      if (res.size != 0)
      {
        Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
        Ogre::Image image;
        std::string extension = fs::extension(fs::path(filename));

        if (extension[0] == '.')
        {
          extension = extension.substr(1, extension.size() - 1);
        }

        try
        {
          image.load(stream, extension);
          Ogre::TextureManager::getSingleton().loadImage(
              filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
        }
        catch (Ogre::Exception& e)
        {
          ROS_ERROR("Could not load texture [%s]: %s", filename.c_str(), e.what());
        }
      }
    }

    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState();

    tex_unit->setTextureName(filename);
  }

  return mat;
}

std::pair<Eigen::Vector3d, size_t> calculate_mesh_centroid(const Ogre::MeshPtr& ogreMesh)
{
  Eigen::Vector3d centroid(0.0, 0.0, 0.0);
  size_t num_vertices {0u};

  for (size_t i = 0; i < ogreMesh->getNumSubMeshes(); ++i)
  {
    const auto& submesh = ogreMesh->getSubMesh(i);
    const auto& vertex_data = submesh->useSharedVertices ? ogreMesh->sharedVertexData : submesh->vertexData;
    const auto& pos_elem = vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
    if (pos_elem == nullptr)
      continue;
    const auto& pos_buffer = vertex_data->vertexBufferBinding->getBuffer(pos_elem->getSource());
    Ogre::HardwareBufferLockGuard pos_vertex_lock(pos_buffer, Ogre::HardwareBuffer::LockOptions::HBL_READ_ONLY);
    auto pos_vertices_data = static_cast<uint8_t*>(pos_vertex_lock.pData);

    const size_t vertex_size = pos_buffer->getVertexSize();
    float* position {nullptr};
    size_t v {0u};
    for (uint8_t* pos_vertex_data = pos_vertices_data; v < vertex_data->vertexCount;
         ++v, pos_vertex_data += vertex_size)
    {
      pos_elem->baseVertexPointerToElement(pos_vertex_data, &position);
      Eigen::Map<Eigen::Vector3f> p(position);
      centroid += p.cast<double>();
      num_vertices++;
    }
  }

  // If the mesh is empty, there is no centroid
  if (num_vertices == 0u)
    return {centroid, 0u};

  centroid /= static_cast<double>(num_vertices);

  return {centroid, num_vertices};
}

Ogre::MeshPtr inflateMesh(const std::string& newMeshName, const Ogre::MeshPtr& ogreMesh, const urdf::Mesh& urdfMesh,
                          const ScaleAndPadding& inflation)
{
  // This is the same algorithm as Mesh::scaleAndPadd from geometric_shapes library, but it extends vertices along
  // normals computed from optimized mesh (all vertices closer than 1 mm are merged into one).
  // https://github.com/moveit/geometric_shapes/blob/noetic-devel/src/shapes.cpp#L378

  if (inflation == ScaleAndPadding(1.0, 0.0) &&
      urdfMesh.scale.x == 1.0 && urdfMesh.scale.y == 1.0 && urdfMesh.scale.z == 1.0)
    return ogreMesh;

  const auto& [centroid, num_vertices] = calculate_mesh_centroid(ogreMesh);

  // If the mesh is empty, do nothing
  if (num_vertices == 0u)
    return ogreMesh;

  // Clone the mesh because we will be changing it
  auto mesh = ogreMesh->clone(newMeshName);

  // When padding is requested, get a temporary mesh with merged close vertices and recomputed normals
  Ogre::MeshPtr opt_mesh;
  MeshOptimizer opt;
  if (inflation.padding != 0.0)
    opt_mesh = opt.optimizeMesh(mesh->clone(newMeshName + "_opt"), 1e-3, 1e3, 1e3);

  Ogre::AxisAlignedBox aabb;
  const Eigen::Vector3d scale(
    inflation.scale * urdfMesh.scale.x, inflation.scale * urdfMesh.scale.y, inflation.scale * urdfMesh.scale.z);
  const auto nonUniformScaling = scale.x() != scale.y() || scale.y() != scale.z();

  for (size_t i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
    Ogre::AxisAlignedBox sub_aabb;

    const auto& submesh = mesh->getSubMesh(i);
    const auto& pos_vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
    const auto& pos_elem = pos_vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
    if (pos_elem == nullptr)
      continue;

    const auto& pos_buffer = pos_vertex_data->vertexBufferBinding->getBuffer(pos_elem->getSource());
    const Ogre::HardwareBufferLockGuard pos_buffer_lock(pos_buffer, Ogre::HardwareBuffer::LockOptions::HBL_NORMAL);
    auto pos_vertices_data = static_cast<uint8_t*>(pos_buffer_lock.pData);
    const size_t pos_vertex_size = pos_buffer->getVertexSize();

    float* position_data {nullptr};
    size_t v {0u};
    for (uint8_t* vertex_start = pos_vertices_data; v < pos_vertex_data->vertexCount;
      ++v, vertex_start += pos_vertex_size)
    {
      pos_elem->baseVertexPointerToElement(vertex_start, &position_data);
      Eigen::Map<Eigen::Vector3f> position(position_data);

      // vector from centroid to the vertex
      const Eigen::Vector3d diff = position.cast<double>() - centroid;

      // Store the scaled coordinate; padding will be added later if needed
      position = (centroid + diff.cwiseProduct(scale)).cast<float>();

      sub_aabb.merge(Ogre::Vector3(position_data));
    }

    // Add padding if needed
    if (inflation.padding != 0.0)
    {
      bool padding_applied {false};

      // Reset the AABB, we'll build it again using the padded points
      sub_aabb.setNull();

      // Try padding along normals if they are available
      if (!opt_mesh.isNull())
      {
        // Only read the normals from optimized mesh in case we need padding; for scaling alone it is not needed

        const auto& opt_submesh = opt_mesh->getSubMesh(i);
        const auto& opt_vertex_data = opt_submesh->useSharedVertices ?
          opt_mesh->sharedVertexData : opt_submesh->vertexData;
        const auto& opt_normals_elem = opt_vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
        if (opt_normals_elem != nullptr)
        {
          const int vertex_buffer_idx = opt_submesh->useSharedVertices ? -1 : static_cast<int>(i);
          const auto& vertex_mapping = opt.mIndexRemaps[vertex_buffer_idx];

          const auto& opt_norm_buffer = opt_vertex_data->vertexBufferBinding->getBuffer(opt_normals_elem->getSource());
          const Ogre::HardwareBufferLockGuard opt_norm_lock(
            opt_norm_buffer, Ogre::HardwareBuffer::LockOptions::HBL_READ_ONLY);
          const auto& opt_vertices_data = static_cast<uint8_t*>(opt_norm_lock.pData);
          const auto opt_vertex_size = opt_norm_buffer->getVertexSize();

          padding_applied = true;

          float* opt_normal_data {nullptr};
          v = 0u;
          for (uint8_t* vertex_start = pos_vertices_data; v < pos_vertex_data->vertexCount;
            ++v, vertex_start += pos_vertex_size)
          {
            pos_elem->baseVertexPointerToElement(vertex_start, &position_data);
            Eigen::Map<Eigen::Vector3f> position(position_data);

            // Find the corresponding vertex in the optimized mesh and read its normal
            const auto opt_vertex = vertex_mapping[v].targetIndex;
            opt_normals_elem->baseVertexPointerToElement(
              opt_vertices_data + opt_vertex * opt_vertex_size, &opt_normal_data);

            Eigen::Vector3d normal = Eigen::Map<Eigen::Vector3f>(opt_normal_data).cast<double>();

            // Transform normal by inverse transpose of scaling matrix if the scaling is nonuniform
            if (nonUniformScaling)
              normal = normal.cwiseProduct(scale.cwiseInverse()).normalized();

            // Add padding in normal direction to the vertex
            position += (normal * inflation.padding).cast<float>();

            sub_aabb.merge(Ogre::Vector3(position_data));
          }
        }
      }

      // Normals are not available, pad in the direction from center
      if (!padding_applied)
      {
        v = 0u;
        for (uint8_t* vertex_start = pos_vertices_data; v < pos_vertex_data->vertexCount;
          ++v, vertex_start += pos_vertex_size)
        {
          pos_elem->baseVertexPointerToElement(vertex_start, &position_data);
          Eigen::Map<Eigen::Vector3f> position(position_data);

          // vector from centroid to the vertex
          const Eigen::Vector3d diff = position.cast<double>() - centroid;

          // Add padding in direction from center to the vertex
          position += (diff.normalized() * inflation.padding).cast<float>();

          sub_aabb.merge(Ogre::Vector3(position_data));
        }
      }
    }

    aabb.merge(sub_aabb);
  }

  mesh->_dirtyState();
  mesh->_setBounds(aabb, true);

  if (mesh->isEdgeListBuilt())
  {
    mesh->freeEdgeList();
    mesh->buildEdgeList();
  }

  mesh->load();
  return mesh;
}

void RobotLink::createEntityForGeometryElement(
  const urdf::LinkConstSharedPtr& link, const urdf::Geometry& geom, const urdf::MaterialSharedPtr& material,
  const urdf::Pose& origin, Ogre::SceneNode* scene_node, Ogre::Entity*& entity, const ScaleAndPadding& inflation)
{
  entity = nullptr; // default in case nothing works.
  Ogre::SceneNode* offset_node = scene_node->createChildSceneNode();

  static unsigned count = 0;
  std::stringstream ss;
  ss << "Robot Link" << ++count;
  std::string entity_name = ss.str();

  Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);

  Ogre::Vector3 offset_position(origin.position.x, origin.position.y, origin.position.z);
  Ogre::Quaternion offset_orientation(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);

  switch (geom.type)
  {
    case urdf::Geometry::SPHERE:
    {
      const auto& sphere = static_cast<const urdf::Sphere&>(geom);
      entity = Shape::createEntity(entity_name, Shape::Sphere, scene_manager_);

      const auto scaleR = static_cast<float>((sphere.radius * inflation.scale + inflation.padding) * 2.0);
      scale = Ogre::Vector3(scaleR, scaleR, scaleR);

      break;
    }
    case urdf::Geometry::BOX:
    {
      const auto& box = static_cast<const urdf::Box&>(geom);
      entity = Shape::createEntity(entity_name, Shape::Cube, scene_manager_);

      const auto scaleX = static_cast<float>(box.dim.x * inflation.scale + 2.0 * inflation.padding);
      const auto scaleY = static_cast<float>(box.dim.y * inflation.scale + 2.0 * inflation.padding);
      const auto scaleZ = static_cast<float>(box.dim.z * inflation.scale + 2.0 * inflation.padding);
      scale = Ogre::Vector3(scaleX, scaleY, scaleZ);

      break;
    }
    case urdf::Geometry::CYLINDER:
    {
      const auto& cylinder = static_cast<const urdf::Cylinder&>(geom);

      Ogre::Quaternion rotX;
      rotX.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
      offset_orientation = offset_orientation * rotX;

      entity = Shape::createEntity(entity_name, Shape::Cylinder, scene_manager_);

      const auto scaleR = static_cast<float>((cylinder.radius * inflation.scale + inflation.padding) * 2.0);
      const auto scaleL = static_cast<float>(cylinder.length * inflation.scale + 2.0 * inflation.padding);
      scale = Ogre::Vector3(scaleR, scaleL, scaleR);

      break;
    }
    case urdf::Geometry::MESH:
    {
      const auto& mesh = static_cast<const urdf::Mesh&>(geom);

      if (mesh.filename.empty())
        return;

      const std::string& model_name = mesh.filename;

      const auto should_inflate = inflation != ScaleAndPadding(1.0, 0.0) ||
        mesh.scale.x != 1.0 || mesh.scale.y != 1.0 || mesh.scale.z != 1.0;

      try
      {
        if (auto ogreMesh = loadMeshFromResource(model_name, should_inflate); ogreMesh.isNull())
        {
          ROS_ERROR("Could not load mesh resource '%s'", model_name.c_str());
        }
        else
        {
          if (should_inflate)
            ogreMesh = inflateMesh(entity_name + "_mesh", ogreMesh, mesh, inflation);
          entity = scene_manager_->createEntity(entity_name, ogreMesh);
        }
      }
      catch (Ogre::InvalidParametersException& e)
      {
        ROS_ERROR("Could not convert mesh resource '%s': %s", model_name.c_str(), e.what());
      }
      catch (Ogre::Exception& e)
      {
        ROS_ERROR("Could not load model '%s': %s", model_name.c_str(), e.what());
      }
      break;
    }
    default:
      ROS_WARN("Unsupported geometry type for element: %d", geom.type);
      break;
  }

  if (entity)
  {
    offset_node->attachObject(entity);
    offset_node->setScale(scale);
    offset_node->setPosition(offset_position);
    offset_node->setOrientation(offset_orientation);

    if (default_material_.isNull() || material)
    {
      // latest used material becomes the default for this link
      default_material_ = getMaterialForLink(link, material);
    }

    for (uint32_t i = 0; i < entity->getNumSubEntities(); ++i)
    {
      // Assign materials only if the submesh does not have one already
      Ogre::SubEntity* sub = entity->getSubEntity(i);
      const std::string& material_name = sub->getMaterialName();

      Ogre::MaterialPtr active, original;
      if (material_name == "BaseWhite" || material_name == "BaseWhiteNoLighting")
        original = default_material_;
      else
        original = sub->getMaterial();

      // create a new material copy for each instance of a RobotLink to allow modification per link
      active = Ogre::MaterialPtr(new Ogre::Material(
          nullptr, material_name, 0, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME));
      *active = *original;
      sub->setMaterial(active);
      materials_[sub] = std::make_pair(active, original);
    }
  }
}

void RobotLink::createCollision(const urdf::LinkConstSharedPtr& link, const std::shared_ptr<ShapeFilter>& shape_filter,
  const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry)
{
  bool valid_collision_found = false;

  for (size_t i = 0; i < link->collision_array.size(); ++i)
  {
    const auto& collision = link->collision_array[i];
    if (collision && collision->geometry)
    {
      if (!shape_filter->considerShape(false, link->name, collision->name, i))
        continue;

      const auto inflation = shape_inflation_registry->getShapeInflation(false, link->name, collision->name, i);
      Ogre::Entity* collision_mesh = nullptr;
      createEntityForGeometryElement(
        link, *collision->geometry, nullptr, collision->origin, collision_node_, collision_mesh, inflation);
      if (collision_mesh)
      {
        collision_meshes_.push_back(collision_mesh);
      }
      valid_collision_found |= collision == link->collision; // don't consider the same geometry twice
    }
  }

  if (!valid_collision_found && link->collision && link->collision->geometry)
  {
    if (shape_filter->considerShape(false, link->name, link->collision->name, 0))
    {
      const auto inflation = shape_inflation_registry->getShapeInflation(false, link->name, link->collision->name, 0);
      Ogre::Entity* collision_mesh = nullptr;
      createEntityForGeometryElement(
        link, *link->collision->geometry, nullptr, link->collision->origin, collision_node_, collision_mesh, inflation);
      if (collision_mesh)
      {
        collision_meshes_.push_back(collision_mesh);
      }
    }
  }

  collision_node_->setVisible(getEnabled());
}

void RobotLink::createVisual(const urdf::LinkConstSharedPtr& link, const std::shared_ptr<ShapeFilter>& shape_filter,
  const std::shared_ptr<ShapeInflationRegistry>& shape_inflation_registry)
{
  bool valid_visual_found = false;

  for (size_t i = 0; i < link->visual_array.size(); ++i)
  {
    const auto& visual = link->visual_array[i];
    if (visual && visual->geometry)
    {
      if (!shape_filter->considerShape(true, link->name, visual->name, i))
        continue;

      const auto inflation = shape_inflation_registry->getShapeInflation(true, link->name, visual->name, i);
      Ogre::Entity* visual_mesh = nullptr;
      createEntityForGeometryElement(
        link, *visual->geometry, visual->material, visual->origin, visual_node_, visual_mesh, inflation);
      if (visual_mesh)
      {
        visual_meshes_.push_back(visual_mesh);
      }
      valid_visual_found |= visual == link->visual; // don't consider the same geometry again
    }
  }

  if (!valid_visual_found && link->visual && link->visual->geometry)
  {
    if (shape_filter->considerShape(true, link->name, link->visual->name, 0))
    {
      const auto inflation = shape_inflation_registry->getShapeInflation(true, link->name, link->visual->name, 0);
      Ogre::Entity* visual_mesh = nullptr;
      createEntityForGeometryElement(link, *link->visual->geometry, link->visual->material, link->visual->origin,
        visual_node_, visual_mesh, inflation);
      if (visual_mesh)
      {
        visual_meshes_.push_back(visual_mesh);
      }
    }
  }

  visual_node_->setVisible(getEnabled());
}

void RobotLink::setTransforms(const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation,
  const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation)
{
  if (visual_node_)
  {
    visual_node_->setPosition(visual_position);
    visual_node_->setOrientation(visual_orientation);
  }

  if (collision_node_)
  {
    collision_node_->setPosition(collision_position);
    collision_node_->setOrientation(collision_orientation);
  }
}

void RobotLink::setToErrorMaterial()
{
  setMaterialMode(material_mode_flags_ | ERROR);
  if (visual_node_)
    visual_node_->setVisible(false);
  if (collision_node_)
    collision_node_->setVisible(false);
}

void RobotLink::setToNormalMaterial()
{
  setMaterialMode(material_mode_flags_ & ~ERROR);
  this->updateVisibility();
}

void RobotLink::setMaterialMode(const unsigned char mode_flags)
{
  if (material_mode_flags_ == mode_flags)
    return; // nothing to change

  material_mode_flags_ = mode_flags;

  if (mode_flags == ORIGINAL)
  {
    for (const auto& [subEntity, material] : materials_)
      subEntity->setMaterial(material.first);
    return;
  }

  Ogre::MaterialPtr material;

  if (mode_flags & ERROR)
  {
    material = Ogre::MaterialManager::getSingleton().getByName(
        "BaseWhiteNoLighting", Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);
  }
  else if (mode_flags & MASK)
  {
    material = mask_material_;
  }
  else if (mode_flags & COLOR)
  {
    material = color_material_;
  }
  else
  {
    throw std::runtime_error("Unsupported mode_flags");
  }

  for (const auto& mesh : visual_meshes_)
    mesh->setMaterial(material);
  for (const auto& mesh : collision_meshes_)
    mesh->setMaterial(material);
}

void RobotLink::setMaskMode()
{
  setMaterialMode(material_mode_flags_ | MASK);
}

void RobotLink::unsetMaskMode()
{
  setMaterialMode(material_mode_flags_ & ~MASK);
}

void RobotLink::setColorMode(const float red, const float green, const float blue)
{
  Ogre::ColourValue color = color_material_->getTechnique(0)->getPass(0)->getDiffuse();
  color.r = red;
  color.g = green;
  color.b = blue;
  color_material_->getTechnique(0)->setAmbient(0.5 * color);
  color_material_->getTechnique(0)->setDiffuse(color);

  setMaterialMode(material_mode_flags_ | COLOR);
}

void RobotLink::unsetColorMode()
{
  setMaterialMode(material_mode_flags_ & ~COLOR);
}

Ogre::Vector3 RobotLink::getPosition() const
{
  if (visual_node_)
    return visual_node_->getPosition();
  if (collision_node_)
    return collision_node_->getPosition();
  return {};
}

Ogre::Quaternion RobotLink::getOrientation() const
{
  if (visual_node_)
    return visual_node_->getOrientation();
  if (collision_node_)
    return collision_node_->getOrientation();
  return {1.f, 0.f, 0.f, 0.f};
}

}
