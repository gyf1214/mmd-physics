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
            virtual void reset() = 0;

            static Body *create(void);
        };

    } /* physics */
} /* mmd */

#endif
