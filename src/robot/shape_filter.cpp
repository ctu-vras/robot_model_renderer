// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <robot_model_renderer/robot/shape_filter.hpp>

#include <cstddef>
#include <memory>
#include <set>
#include <string>

#include <cras_cpp_common/set_utils.hpp>

namespace robot_model_renderer
{

struct ShapeFilter::Implementation
{
  bool visualAllowed {true};
  bool collisionAllowed {false};
  std::set<std::string> ignoredShapes;
  std::set<std::string> onlyShapes;
};

ShapeFilter::ShapeFilter(const bool visualAllowed, const bool collisionAllowed) : data(new Implementation)
{
  this->data->visualAllowed = visualAllowed;
  this->data->collisionAllowed = collisionAllowed;
}

ShapeFilter::~ShapeFilter() = default;

bool ShapeFilter::isVisualAllowed() const
{
  return this->data->visualAllowed;
}

bool ShapeFilter::isCollisionAllowed() const
{
  return this->data->collisionAllowed;
}

void ShapeFilter::setIgnoreShapes(const std::set<std::string>& shapes)
{
  this->data->ignoredShapes = shapes;
}

void ShapeFilter::setOnlyShapes(const std::set<std::string>& shapes)
{
  this->data->onlyShapes = shapes;
}

bool ShapeFilter::considerLink(const std::string& linkName) const
{
  if (!this->data->onlyShapes.empty())
    return this->data->onlyShapes.find(linkName) != this->data->onlyShapes.end();

  return this->data->ignoredShapes.find(linkName) == this->data->ignoredShapes.end();
}

bool ShapeFilter::considerShape(const bool isVisual, const std::string& linkName, const std::string& shapeName,
                                const size_t shapeIndex) const
{
  if ((!this->data->visualAllowed && isVisual) || (!this->data->collisionAllowed && !isVisual))
    return false;

  // Fast track for empty filters
  if (this->data->onlyShapes.empty() && this->data->ignoredShapes.empty())
    return true;

  const std::string typeSep = isVisual ? ":visual:" : ":collision:";
  const auto shapeIndexStr = std::to_string(shapeIndex);

  const std::set<std::string> shapeNames =
  {
    linkName,
    "*::" + shapeName,
    linkName + "::" + shapeIndexStr,
    linkName + "::" + shapeName,
    "*" + typeSep + shapeName,
    linkName + typeSep + shapeIndexStr,
    linkName + typeSep + shapeName,
  };

  // if onlyShapes is nonempty, make sure this shape is allowed
  if (!this->data->onlyShapes.empty() && cras::isSetIntersectionEmpty(shapeNames, this->data->onlyShapes))
    return false;

  // if the link is ignored, do not consider it
  if (!cras::isSetIntersectionEmpty(shapeNames, this->data->ignoredShapes))
    return false;

  return true;
}

const std::set<std::string>& ShapeFilter::ignoredShapes() const
{
  return this->data->ignoredShapes;
}

const std::set<std::string>& ShapeFilter::onlyShapes() const
{
  return this->data->onlyShapes;
}

}
