#ifndef __MMD_PHYSICS_MOTION
#define __MMD_PHYSICS_MOTION

#include "common.hpp"
#include "mmd/pmx.hpp"
#include "mmd/vmd.hpp"

namespace mmd {
    namespace physics {

        class MMDAPI Motion {
        public:
            virtual ~Motion();
            virtual void loadModel(const pmx::Model *model) = 0;
            virtual void loadMotion(const vmd::Motion *motion) = 0;
            virtual void loadBody(void) = 0;

            virtual void resetPhysics(void) = 0;
            virtual void resetPose(void) = 0;
            virtual void resetMotion(void) = 0;
            virtual void resetModel(void) = 0;
            virtual void reset(void) = 0;

            virtual void updateGlobal(const glm::mat4 &m) = 0;
            virtual void updateKey(float frame) = 0;
            virtual void updatePhysics(float tick) = 0;

            virtual glm::mat4 skin(int index) = 0;
            virtual float face(int index) = 0;

            static Motion *create(void);
        };

    } /* physics */
} /* mmd */

#endif
