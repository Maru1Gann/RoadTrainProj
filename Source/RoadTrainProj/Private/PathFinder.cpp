
#include "PathFinder.h"
#include "PathNode.h"
#include "LandscapeManager.h"

#include <limits>
const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles

FPathFinder::FPathFinder( ALandscapeManager* pLM ) : pLM( pLM )
{
}


