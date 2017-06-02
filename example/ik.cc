#include "mmd-physics/ik.hpp"
#include "../src/ext.hpp"

using namespace mmd::physics;
using namespace glm;

int main() {
    IK *ik = IK::gradient(0.01f, 1e-7);
    ik->pushBone(vec3(0.0f, 0.0f, 0.0f));
    ik->pushBone(vec3(0.0f, 1.0f, 0.0f));
    ik->pushBone(vec3(0.0f, 2.0f, 0.0f));
    ik->solve(vec3(0.5f, 0.5f, 0.0f));
    LOG << ik->getDirection(0);
    LOG << ik->getDirection(1);

    return 0;
}
