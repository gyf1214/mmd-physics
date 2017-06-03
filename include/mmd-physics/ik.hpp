#ifndef __MMD_PHYSICS_IK
#define __MMD_PHYSICS_IK

#include "common.hpp"

namespace mmd {
    namespace physics {

        class MMDAPI IK {
        public:
            virtual ~IK();
            virtual void reset(void) = 0;
            virtual void pushBone(const glm::vec3 &position) = 0;
            virtual void solve(const glm::vec3 &target) = 0;
            virtual glm::vec3 getDirection(int index) = 0;
            virtual glm::mat4 getRotation(int index) = 0;

            static IK *gradient(float alpha, int loops);
        };

    } /* physics */
} /* mmd */

#endif
