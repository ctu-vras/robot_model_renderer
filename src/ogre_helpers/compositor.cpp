// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <robot_model_renderer/ogre_helpers/compositor.hpp>

#include <cstdint>
#include <string>

#include <OgreCompositionPass.h>
#include <OgreCompositionTargetPass.h>
#include <OgreCompositionTechnique.h>
#include <OgreCompositorInstance.h>

namespace robot_model_renderer
{

Ogre::CompositionTargetPass* getTargetPass(Ogre::CompositionTechnique* technique, const std::string& outputName)
{
  if (technique == nullptr)
    return nullptr;

  if (outputName.empty())
    return technique->getOutputTargetPass();

  auto it = technique->getTargetPassIterator();
  const auto checkOutputName = [outputName](const Ogre::CompositionTargetPass* t)
  {
    return t->getOutputName() == outputName;
  };

  const auto tpIt = std::find_if(it.begin(), it.end(), checkOutputName);
  if (tpIt == it.end())
    return nullptr;

  return *tpIt;
}

Ogre::CompositionTargetPass* getTargetPass(Ogre::CompositorInstance* instance, const std::string& outputName)
{
  if (instance == nullptr)
    return nullptr;
  return getTargetPass(instance->getTechnique(), outputName);
}

Ogre::CompositionPass* getPass(
  Ogre::CompositionTechnique* technique, const std::string& outputName, const uint32_t identifier)
{
  const auto targetPass = getTargetPass(technique, outputName);
  if (targetPass == nullptr)
    return nullptr;

  auto it = targetPass->getPassIterator();
  const auto checkId = [identifier](const Ogre::CompositionPass* t)
  {
    return t->getIdentifier() == identifier;
  };

  const auto passIt = std::find_if(it.begin(), it.end(), checkId);
  if (passIt == it.end())
    return nullptr;

  return *passIt;
}

Ogre::CompositionPass* getPass(
  Ogre::CompositorInstance* instance, const std::string& outputName, const uint32_t identifier)
{
  if (instance == nullptr)
    return nullptr;
  return getPass(instance->getTechnique(), outputName, identifier);
}

}
