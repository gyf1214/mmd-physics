#ifndef __MMD_PHYSICS_ARMATURE
#define __MMD_PHYSICS_ARMATURE

#include "common.hpp"
#include "mmd/pmx.hpp"

namespace mmd {
    namespace physics {

        class MMDAPI Armature {
        public:
            virtual ~Armature();
            virtual void loadModel(pmx::Model *model) = 0;
            virtual void reset(void) = 0;
            virtual void applyLocal(int index, const glm::mat4 &m) = 0;
            virtual void solveIK(void) = 0;
            virtual int getSize(void) = 0;
            virtual glm::mat4 skin(int index) = 0;

            static Armature *create(void);
        };

    } /* physics */
} /* mmd */

#endif
