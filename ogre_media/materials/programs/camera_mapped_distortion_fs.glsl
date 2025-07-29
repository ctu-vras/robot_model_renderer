// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// Mapping of undistorted to distorted uv coordinates.
uniform sampler2D distortionMap;

uniform vec4 backgroundColor;

void main()
{
  vec2 uvDistorted = texture2D(distortionMap, gl_TexCoord[0].xy).xy;

  if (uvDistorted.x < 0.0 || uvDistorted.y < 0.0)
    gl_FragColor = backgroundColor;
  else
    gl_FragColor = texture2D(RT, uvDistorted);
}

