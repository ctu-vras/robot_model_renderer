// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <robot_model_renderer/robot/shape_inflation_registry.h>

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <cras_cpp_common/optional.hpp>

namespace robot_model_renderer
{

struct ShapeInflationRegistry::Implementation
{
  ScaleAndPadding defaultInflation {1.0, 0.0};
  cras::optional<ScaleAndPadding> defaultVisualInflation {cras::nullopt};
  cras::optional<ScaleAndPadding> defaultCollisionInflation {cras::nullopt};
  std::unordered_map<std::string, ScaleAndPadding> perShapeInflation;
};

ScaleAndPadding::ScaleAndPadding(const double scale, const double padding) : scale(scale), padding(padding)
{
}

bool ScaleAndPadding::operator==(const ScaleAndPadding& other) const
{
  return this->scale == other.scale && this->padding == other.padding;
}

bool ScaleAndPadding::operator!=(const ScaleAndPadding& other) const
{
  return !(*this == other);
}

ShapeInflationRegistry::ShapeInflationRegistry(const double defaultScale, const double defaultPadding) :
  ShapeInflationRegistry(ScaleAndPadding(defaultScale, defaultPadding))
{
}

ShapeInflationRegistry::ShapeInflationRegistry(const ScaleAndPadding& scalePadding) : data(new Implementation())
{
  this->data->defaultInflation = scalePadding;
}

ShapeInflationRegistry::~ShapeInflationRegistry() = default;

void ShapeInflationRegistry::setDefaultVisualInflation(const ScaleAndPadding& scalePadding)
{
  this->data->defaultVisualInflation = scalePadding;
}

void ShapeInflationRegistry::setDefaultCollisionInflation(const ScaleAndPadding& scalePadding)
{
  this->data->defaultCollisionInflation = scalePadding;
}

void ShapeInflationRegistry::addPerShapeInflation(const std::string& name, const ScaleAndPadding& scalePadding)
{
  this->data->perShapeInflation[name] = scalePadding;
}

ScaleAndPadding ShapeInflationRegistry::getShapeInflation(const bool isVisual, const std::string& linkName,
                                                          const std::string& shapeName, const size_t shapeIndex) const
{
  const auto defaultInflation = isVisual ?
    this->data->defaultVisualInflation.value_or(this->data->defaultInflation) :
    this->data->defaultCollisionInflation.value_or(this->data->defaultInflation);

  // Fast track for empty per-shape config
  if (this->data->perShapeInflation.empty())
    return defaultInflation;

  const std::string typeSep = isVisual ? ":visual:" : ":collision:";
  const auto shapeIndexStr = std::to_string(shapeIndex);

  const std::vector<std::string> shapeNames =
  {
    linkName + typeSep + shapeName,
    linkName + "::" + shapeName,
    "*" + typeSep + shapeName,
    "*::" + shapeName,
    linkName + typeSep + shapeIndexStr,
    linkName + "::" + shapeIndexStr,
    linkName,
  };

  for (const auto& name : shapeNames)
  {
    if (const auto it = this->data->perShapeInflation.find(name); it != this->data->perShapeInflation.end())
    {
      return it->second;
    }
  }

  return defaultInflation;
}

const ScaleAndPadding& ShapeInflationRegistry::defaultInflation() const
{
  return this->data->defaultInflation;
}

const cras::optional<ScaleAndPadding>& ShapeInflationRegistry::defaultVisualInflation() const
{
  return this->data->defaultVisualInflation;
}

const cras::optional<ScaleAndPadding>& ShapeInflationRegistry::defaultCollisionInflation() const
{
  return this->data->defaultCollisionInflation;
}

const std::unordered_map<std::string, ScaleAndPadding>& ShapeInflationRegistry::perShapeInflation() const
{
  return this->data->perShapeInflation;
}

}
