// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>

#include <cras_cpp_common/optional.hpp>

namespace robot_model_renderer
{

struct ScaleAndPadding
{
  double scale;
  double padding;

  explicit ScaleAndPadding(double scale = 1.0, double padding = 0.0);

  bool operator==(const ScaleAndPadding& other) const;
  bool operator!=(const ScaleAndPadding& other) const;
};

class ShapeInflationRegistry
{
public:
  explicit ShapeInflationRegistry(double defaultScale = 1.0, double defaultPadding = 0.0);
  explicit ShapeInflationRegistry(const ScaleAndPadding& scalePadding);
  virtual ~ShapeInflationRegistry();

  virtual void setDefaultVisualInflation(const ScaleAndPadding& scalePadding);
  virtual void setDefaultCollisionInflation(const ScaleAndPadding& scalePadding);
  virtual void addPerShapeInflation(const std::string& name, const ScaleAndPadding& scalePadding);

  virtual ScaleAndPadding getShapeInflation(
    bool isVisual, const std::string& linkName, const std::string& shapeName, size_t shapeIndex) const;

  virtual const ScaleAndPadding& defaultInflation() const;
  virtual const cras::optional<ScaleAndPadding>& defaultVisualInflation() const;
  virtual const cras::optional<ScaleAndPadding>& defaultCollisionInflation() const;
  virtual const std::unordered_map<std::string, ScaleAndPadding>& perShapeInflation() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> data;
};

}
