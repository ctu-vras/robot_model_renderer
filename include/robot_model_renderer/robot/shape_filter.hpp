// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <set>

namespace robot_model_renderer
{

/**
 * \brief Filter of shapes (visual/collision elements).
 *
 * The methods setIgnoreShapes() and setOnlyShapes() accept "shape name templates". These can be:
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
class ShapeFilter
{
public:
  /**
   * \brief Construct a default filter that accepts everything.
   *
   * \param[in] visualAllowed Whether visual elements are generally allowed.
   * \param[in] collisionAllowed Whether collision elements are generally allowed.
   */
  explicit ShapeFilter(bool visualAllowed = true, bool collisionAllowed = false);

  virtual ~ShapeFilter();

  /**
   * \return Whether visual elements are generally allowed.
   */
  virtual bool isVisualAllowed() const;

  /**
   * \return Whether collision elements are generally allowed.
   */
  virtual bool isCollisionAllowed() const;

  /**
   * \param[in] shapes "Shape name templates" of the shapes which will be ignored.
   */
  virtual void setIgnoreShapes(const std::set<std::string>& shapes);

  /**
   * \param[in] shapes "Shape name templates" of white-listed shapes that will be allowed.
   */
  virtual void setOnlyShapes(const std::set<std::string>& shapes);

  /**
   * \brief Check whether the given link is allowed.
   *
   * \param[in] linkName The link name to query.
   * \return Whether the link is allowed.
   */
  virtual bool considerLink(const std::string& linkName) const;

  /**
   * \brief Check whether the given shape is allowed.
   *
   * \param[in] isVisual Whether the shape is a visual (true) or collisions (false).
   * \param[in] linkName The link name to query.
   * \param[in] shapeName Name of the shape to query (can be empty if it has no name).
   * \param[in] shapeIndex Ordinal number of the shape in its link.
   * \return Whether the shape is allowed.
   */
  virtual bool considerShape(
    bool isVisual, const std::string& linkName, const std::string& shapeName, size_t shapeIndex) const;

protected:
  virtual const std::set<std::string>& ignoredShapes() const;
  virtual const std::set<std::string>& onlyShapes() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> data;  //!< PIMPL
};

}
