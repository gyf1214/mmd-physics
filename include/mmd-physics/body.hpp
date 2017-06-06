#ifndef __MMD_PHYSICS_BODY
#define __MMD_PHYSICS_BODY

#include "common.hpp"
#include "armature.hpp"

namespace mmd {
    namespace physics {

        class Body {
        public:
            virtual ~Body();
            virtual void loadModel(const pmx::Model *model) = 0;
            virtual void bindArmature(Armature *armature) = 0;
            virtual void reset(void) = 0;
            virtual void resetPose(void) = 0;
            virtual void applyBone(void) = 0;
            virtual void stepSimulation(float tick) = 0;
            virtual void updateBone(void) = 0;
            virtual void update(float tick) = 0;

            static Body *create(void);
        };

    } /* physics */
} /* mmd */

#endif
