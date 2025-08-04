#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <set>

namespace robot_model_renderer
{

class ShapeFilter
{
public:
  explicit ShapeFilter(bool visualAllowed = true, bool collisionAllowed = false);
  virtual ~ShapeFilter();

  virtual bool isVisualAllowed() const;
  virtual bool isCollisionAllowed() const;

  virtual void setIgnoreShapes(const std::set<std::string>& shapes);
  virtual void setOnlyShapes(const std::set<std::string>& shapes);

  virtual bool considerLink(const std::string& linkName) const;
  virtual bool considerShape(
    bool isVisual, const std::string& linkName, const std::string& shapeName, size_t shapeIndex) const;

protected:
  virtual const std::set<std::string>& ignoredShapes() const;
  virtual const std::set<std::string>& onlyShapes() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> data;
};

}
