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

/**
 * \brief A structure holding the configuration of scaling and padding for a single shape.
 */
struct ScaleAndPadding
{
  double scale;  //!< Scaling (1.0 = no scaling).
  double padding;  //!< Padding (meters).

  /**
   * \param[in] scale The scaling.
   * \param[in] padding The padding (meters).
   */
  explicit ScaleAndPadding(double scale = 1.0, double padding = 0.0);

  bool operator==(const ScaleAndPadding& other) const;
  bool operator!=(const ScaleAndPadding& other) const;
};

/**
 * \brief A registry of scaling and padding settings for shapes (visual/collision elements).
 *
 * The methods in this class accept "shape name templates". These can be:
 *
 * - `link` (match all shapes in the link with name `link`)
 * - `*::shape` (match all shapes with name `shape` in any link)
 * - `link::shape#` (match the `shape#`-th visual or collision in the given link)
 * - `link::shape` (match shape with name `shape` in link with name `link`)
 * - `*:visual:shape` (match visual with name `shape` in any link)
 * - `link:visual:shape#` (match `shape#`-th visual in link with name `link`)
 * - `link:visual:shape` (match visual with name `shape` in link with name `link`)
 * - `*:collision:shape` (match collision with name `shape` in any link)
 * - `link:collision:shape#` (match `shape#`-th collision in link with name `link`)
 * - `link:collision:shape` (match collision with name `shape` in link with name `link`)
 */
class ShapeInflationRegistry
{
public:
  /**
   * \param[in] defaultScale Default scaling to be used for shapes without explicit override.
   * \param[in] defaultPadding Default padding to be used for shapes without explicit override (meters).
   */
  explicit ShapeInflationRegistry(double defaultScale = 1.0, double defaultPadding = 0.0);

  /**
   * \param[in] scalePadding Default scale and padding to be used for shapes without explicit override.
   */
  explicit ShapeInflationRegistry(const ScaleAndPadding& scalePadding);

  virtual ~ShapeInflationRegistry();

  /**
   * \param[in] scalePadding Default scale and padding to be used for visuals without explicit override.
   */
  virtual void setDefaultVisualInflation(const ScaleAndPadding& scalePadding);

  /**
   * \param[in] scalePadding Default scale and padding to be used for collisions without explicit override.
   */
  virtual void setDefaultCollisionInflation(const ScaleAndPadding& scalePadding);

  /**
   * \brief Add explicit override of scale and padding for the given shape(s).
   *
   * \param[in] name The "shape name template" defining which shape(s) to match.
   * \param[in] scalePadding The scale and padding the matched shape(s).
   */
  virtual void addPerShapeInflation(const std::string& name, const ScaleAndPadding& scalePadding);

  /**
   * \brief Get the scale and padding of the given shape.
   *
   * \param[in] isVisual Whether the shape is visual (true) or collision (false).
   * \param[in] linkName Name of the link that holds the shape.
   * \param[in] shapeName Name of the shape (can be empty if it has no name).
   * \param[in] shapeIndex Ordinal number of the shape in its link.
   * \return The appropriate scale and padding.
   */
  virtual ScaleAndPadding getShapeInflation(
    bool isVisual, const std::string& linkName, const std::string& shapeName, size_t shapeIndex) const;

  /**
   * \return Default scale and padding to be used for shapes without explicit override.
   */
  virtual const ScaleAndPadding& defaultInflation() const;

  /**
   * \return Default scale and padding to be used for visuals without explicit override.
   */
  virtual const cras::optional<ScaleAndPadding>& defaultVisualInflation() const;

  /**
   * \return Default scale and padding to be used for collisions without explicit override.
   */
  virtual const cras::optional<ScaleAndPadding>& defaultCollisionInflation() const;

  /**
   * \return The configured per-shape scale and padding overrides.
   */
  virtual const std::unordered_map<std::string, ScaleAndPadding>& perShapeInflation() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> data;  //!< PIMPL
};

}
