// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <cstdint>
#include <string>

#include <OgrePrerequisites.h>

namespace Ogre
{

class CompositionPass;
class CompositionTargetPass;
class CompositionTechnique;
class CompositorInstance;

}

namespace robot_model_renderer
{

/**
 * \brief Find a composition target pass in the given technique that is identified by its targetPass output name.
 * \param[in] technique The composition technique to search.
 * \param[in] outputName The name of the targetPass. If empty, finds the output pass.
 * \return The selected target pass, or nullptr if not found.
 */
Ogre::CompositionTargetPass* getTargetPass(Ogre::CompositionTechnique* technique, const std::string& outputName);

/**
 * \brief Find a composition target pass in the given compositor instance that is identified by its targetPass
 *        output name.
 * \param[in] instance The compositor instance to search.
 * \param[in] outputName The name of the targetPass. If empty, finds the output pass.
 * \return The selected target pass, or nullptr if not found.
 */
Ogre::CompositionTargetPass* getTargetPass(Ogre::CompositorInstance* instance, const std::string& outputName);

/**
 * \brief Find a composition pass in the given technique that is identified by its targetPass output name and the
 *        identifier field.
 * \param[in] technique The composition technique to search.
 * \param[in] outputName The name of the targetPass. If empty, finds the output pass.
 * \param[in] identifier The identifier of the pass.
 * \return The selected pass, or nullptr if not found.
 */
Ogre::CompositionPass* getPass(
  Ogre::CompositionTechnique* technique, const std::string& outputName, uint32_t identifier);

/**
 * \brief Find a composition pass in the given compositor instance that is identified by its targetPass output name and
 *        the identifier field.
 * \param[in] instance The compositor instance to search.
 * \param[in] outputName The name of the targetPass. If empty, finds the output pass.
 * \param[in] identifier The identifier of the pass.
 * \return The selected pass, or nullptr if not found.
 */
Ogre::CompositionPass* getPass(
  Ogre::CompositorInstance* instance, const std::string& outputName, uint32_t identifier);

}
