// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;
uniform int invert_alpha;

void main()
{
  vec4 color = texture2D(RT, gl_TexCoord[0].xy);

  if (invert_alpha == 0)
  {
    gl_FragColor.xyz = 1.0 - color.xyz;
    gl_FragColor.a = color.a;
  }
  else
  {
    gl_FragColor = 1.0 - color;
  }
}

